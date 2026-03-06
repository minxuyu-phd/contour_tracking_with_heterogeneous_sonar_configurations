#!/usr/bin/env python3
"""
MSIS Adaptive Scanner - Step angle and scan range adaptive control

This script implements an adaptive scanning algorithm for the MSIS (Mechanical Scanning Imaging Sonar)
that automatically adjusts:
  1. Step angle based on detection density in a sliding window
  2. Scan range based on SBES (Single Beam Echo Sounder) detections (optional)

Step Angle Adaptation:
    Detection density eta_c = detections in window / window samples

    eta_c > 0.6  -> Fine scan (step_multiplier=1, 0.5 deg)
    eta_c < 0.4  -> Coarse scan (step_multiplier=3, 1.5 deg)
    0.4 <= eta_c <= 0.6 -> Maintain current mode (hysteresis)

Scan Range Adaptation (--enable-sbes):
    Based on left/right SBES detections and a state machine with stability threshold N_S:

    State  | Condition                           | Scan Range
    -------|-------------------------------------|---------------------------
    FULL   | l_counter < N_S AND r_counter < N_S | [-180, 180] deg
    BOTH   | l_counter >= N_S AND r_counter >= N_S | [gap_left, gap_right]
    L_UNC  | l_counter < N_S AND r_counter >= N_S | [-alpha_safe, gap_right]
    R_UNC  | l_counter >= N_S AND r_counter < N_S | [gap_left, alpha_safe]

    Counters increment on consecutive detections, reset on no detection.

Usage:
    python msis_adaptive_scanner.py [options]
    python msis_adaptive_scanner.py --enable-sbes --verbose

Requires:
    nng_comm module (contour_tracker/nng_comm)
"""

import argparse
import json
import os
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
from nng_comm.scripts import Subscriber, Requester


def load_jsonc(path: str) -> dict:
    """加载 JSONC 文件（支持 // 注释，正确跳过字符串内的 //）"""
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    result, i, in_string = [], 0, False
    while i < len(content):
        c = content[i]
        if c == '\\' and in_string:
            result.append(c); result.append(content[i + 1]); i += 2; continue
        if c == '"':
            in_string = not in_string
        if not in_string and c == '/' and i + 1 < len(content) and content[i + 1] == '/':
            while i < len(content) and content[i] != '\n':
                i += 1
            continue
        result.append(c); i += 1
    return json.loads(''.join(result))


@dataclass
class AdaptiveConfig:
    """Configuration parameters for adaptive scanning algorithm

    Scan range modes (based on sim_multi_sonar.py state machine):
      - FULL:  [-180, 180] deg - both sides uncertain
      - BOTH:  [gap_left, gap_right] - both SBES confirmed obstacles
      - L_UNC: [-alpha_safe, gap_right] - left uncertain, right confirmed
      - R_UNC: [gap_left, alpha_safe] - right uncertain, left confirmed

    The safety half-angle (alpha_safe) limits forward scan when one side is uncertain.
    Default values based on Ping360 MSS and Ping2 SBES configuration:
      - SBES beam width: 25 deg
      - SBES mounting angles: -75 deg (left), +75 deg (right)
      - Gap region: [-62.5, +62.5] deg (SBES coverage gap)
      - Safety half-angle: 28 deg
    """
    # Sliding window parameters
    window_duration: float = 2.0  # seconds
    sample_rate: float = 10.0  # Hz (MSIS data frequency)

    # Density thresholds for mode switching (with hysteresis)
    density_high: float = 0.6  # Switch to fine scan above this (M_S in paper)
    density_low: float = 0.4   # Switch to coarse scan below this (1 - M_S)

    # Step multipliers (base step is 0.5 degrees for Ping360)
    step_multiplier_fine: int = 1   # 0.5 deg (DZ_FINE)
    step_multiplier_coarse: int = 3  # 1.5 deg (DZ_COARSE)

    # Detection threshold for beam_data
    intensity_threshold: int = 100  # 0-255

    # Scan range limits (degrees) - default for standalone MSIS operation
    rotation_min: float = -45.0
    rotation_max: float = 45.0

    # Safety half-angle (degrees) - used when SBES provides partial coverage
    # When one side SBES detects obstacles and the other doesn't, the scan
    # range is limited to [-alpha_safe, gap_right] or [gap_left, alpha_safe]
    alpha_safe: float = 28.0  # Safety half-angle (ALPHA_SAFE in paper)

    # SBES gap region boundaries (degrees) - where SBES cannot cover
    # Based on SBES mounting at +/-75 deg with 25 deg beam width
    gap_left: float = -62.5   # Left boundary of SBES coverage gap
    gap_right: float = 62.5   # Right boundary of SBES coverage gap

    # SBES scan range switching parameters
    n_s: int = 3  # Stability threshold (consecutive detection count)
    sbes_max_range: float = 100.0  # SBES max detection range (meters)

    @property
    def window_samples(self) -> int:
        """Number of samples in sliding window"""
        return int(self.window_duration * self.sample_rate)


class StepAdaptiveScanner:
    """Core adaptive scanning algorithm based on sliding window detection density"""

    def __init__(self, config: AdaptiveConfig):
        self.cfg = config
        self.detection_history: deque = deque(maxlen=config.window_samples)
        self.current_mode: str = "COARSE"
        self.current_step_multiplier: int = config.step_multiplier_coarse
        self.total_samples: int = 0
        self.total_detections: int = 0

    def add_detection(self, detected: bool) -> None:
        """Add a detection result to the sliding window"""
        self.detection_history.append(detected)
        self.total_samples += 1
        if detected:
            self.total_detections += 1

    def compute_detection_density(self) -> float:
        """Compute detection density (eta_c) from sliding window"""
        if len(self.detection_history) == 0:
            return 0.0
        return sum(self.detection_history) / len(self.detection_history)

    def update_scan_mode(self) -> Tuple[bool, str, int]:
        """
        Update scan mode based on detection density with hysteresis

        Returns:
            (mode_changed, current_mode, step_multiplier)
        """
        eta_c = self.compute_detection_density()
        old_mode = self.current_mode

        if eta_c > self.cfg.density_high:
            self.current_mode = "FINE"
            self.current_step_multiplier = self.cfg.step_multiplier_fine
        elif eta_c < self.cfg.density_low:
            self.current_mode = "COARSE"
            self.current_step_multiplier = self.cfg.step_multiplier_coarse
        # else: maintain current mode (hysteresis region)

        mode_changed = (old_mode != self.current_mode)
        return mode_changed, self.current_mode, self.current_step_multiplier

    def get_stats(self) -> dict:
        """Get current statistics"""
        return {
            "window_size": len(self.detection_history),
            "window_max": self.cfg.window_samples,
            "detection_density": self.compute_detection_density(),
            "current_mode": self.current_mode,
            "step_multiplier": self.current_step_multiplier,
            "total_samples": self.total_samples,
            "total_detections": self.total_detections,
        }


class MSISSubscriber:
    """MSIS data subscriber using nng_comm Subscriber"""

    DEFAULT_ADDRESS = "tcp://192.168.5.11:5560"

    def __init__(self, address: Optional[str] = None):
        self.address = address or self.DEFAULT_ADDRESS
        self.socket: Optional[Subscriber] = None
        self._recv_timeout_ms: int = 1000

    def connect(self, recv_timeout_ms: int = 1000) -> None:
        """Connect to MSIS publisher"""
        self._recv_timeout_ms = recv_timeout_ms
        self.socket = Subscriber()
        self.socket.init(self.address)

    def receive(self) -> Optional[dict]:
        """Receive and parse MSIS message, returns None on timeout"""
        if self.socket is None:
            raise RuntimeError("Not connected")
        return self.socket.receive(timeout_ms=self._recv_timeout_ms)

    def close(self) -> None:
        """Close the socket"""
        if self.socket:
            self.socket.close()
            self.socket = None


class SBESSubscriber:
    """SBES (Echosounder) data subscriber for left and right sensors"""

    LEFT_ADDRESS = "tcp://192.168.5.11:5561"
    RIGHT_ADDRESS = "tcp://192.168.5.11:5562"

    def __init__(self, left_address: Optional[str] = None,
                 right_address: Optional[str] = None):
        self.left_address = left_address or self.LEFT_ADDRESS
        self.right_address = right_address or self.RIGHT_ADDRESS
        self.left_socket: Optional[Subscriber] = None
        self.right_socket: Optional[Subscriber] = None
        self._recv_timeout_ms: int = 100

    def connect(self, recv_timeout_ms: int = 100) -> None:
        """Connect to both SBES publishers"""
        self._recv_timeout_ms = recv_timeout_ms
        self.left_socket = Subscriber()
        self.left_socket.init(self.left_address)

        self.right_socket = Subscriber()
        self.right_socket.init(self.right_address)

    def receive_left(self) -> Optional[dict]:
        """Receive and parse left SBES message, returns None on timeout"""
        if self.left_socket is None:
            raise RuntimeError("Not connected")
        return self.left_socket.receive(timeout_ms=self._recv_timeout_ms)

    def receive_right(self) -> Optional[dict]:
        """Receive and parse right SBES message, returns None on timeout"""
        if self.right_socket is None:
            raise RuntimeError("Not connected")
        return self.right_socket.receive(timeout_ms=self._recv_timeout_ms)

    def close(self) -> None:
        """Close both sockets"""
        if self.left_socket:
            self.left_socket.close()
            self.left_socket = None
        if self.right_socket:
            self.right_socket.close()
            self.right_socket = None


class MSISConfigurator:
    """MSIS configuration sender using nng_comm Requester"""

    DEFAULT_ADDRESS = "tcp://192.168.5.11:7777"

    def __init__(self, address: Optional[str] = None, request_timeout_ms: int = 5000):
        self.address = address or self.DEFAULT_ADDRESS
        self.request_timeout_ms = request_timeout_ms

    def send_config(self, rotation_min: float, rotation_max: float,
                    step_multiplier: int) -> dict:
        """Send MSIS configuration and return response"""
        cmd = {
            "command": "configure_msis",
            "rotation_min": float(rotation_min),
            "rotation_max": float(rotation_max),
            "step_multiplier": int(step_multiplier),
        }

        req = Requester()
        req.init(self.address)
        try:
            reply = req.request(cmd, timeout_ms=self.request_timeout_ms)
            return reply if reply is not None else {}
        finally:
            req.close()

    def get_config(self) -> dict:
        """Get current MSIS configuration"""
        cmd = {"command": "get_msis_config"}

        req = Requester()
        req.init(self.address)
        try:
            reply = req.request(cmd, timeout_ms=self.request_timeout_ms)
            return reply if reply is not None else {}
        finally:
            req.close()


def process_beam_data(beam_data, threshold: int = 100) -> bool:
    """
    Process MSIS beam_data to determine if detection occurred

    Args:
        beam_data: Can be:
            - dict with 'bytes' key containing list of uint8 values
            - list of uint8 intensity values
            - bytes object
        threshold: Detection threshold (0-255)

    Returns:
        True if max intensity exceeds threshold
    """
    if not beam_data:
        return False

    # Handle dict format: {"bytes": [...]}
    if isinstance(beam_data, dict):
        beam_data = beam_data.get('bytes', [])

    if isinstance(beam_data, bytes):
        beam_data = list(beam_data)

    if isinstance(beam_data, str):
        # Handle base64 encoded data - decode it
        try:
            import base64
            decoded = base64.b64decode(beam_data)
            beam_data = list(decoded)
        except Exception:
            return False

    if not beam_data or not isinstance(beam_data, list):
        return False

    return max(beam_data) > threshold


class ScanRangeStateMachine:
    """SBES-based scan range state machine

    State definitions:
      - FULL:  l_counter < N_S AND r_counter < N_S  -> [-180, 180] deg
      - BOTH:  l_counter >= N_S AND r_counter >= N_S -> [gap_left, gap_right]
      - L_UNC: l_counter < N_S AND r_counter >= N_S -> [-alpha_safe, gap_right]
      - R_UNC: l_counter >= N_S AND r_counter < N_S -> [gap_left, alpha_safe]
    """

    def __init__(self, config: AdaptiveConfig):
        self.cfg = config
        self.l_counter: int = 0  # Left SBES consecutive detection count
        self.r_counter: int = 0  # Right SBES consecutive detection count
        self.state: str = "FULL"

    def update(self, left_detect: bool, right_detect: bool) -> Tuple[bool, str, float, float]:
        """
        Update state machine based on SBES detections

        Args:
            left_detect: True if left SBES detected obstacle
            right_detect: True if right SBES detected obstacle

        Returns:
            (state_changed, state_label, zeta_min, zeta_max)
        """
        # Update counters: increment on detection, reset on no detection
        self.l_counter = self.l_counter + 1 if left_detect else 0
        self.r_counter = self.r_counter + 1 if right_detect else 0

        old_state = self.state

        # State transition logic
        if self.l_counter >= self.cfg.n_s and self.r_counter >= self.cfg.n_s:
            self.state = "BOTH"
            zeta_min, zeta_max = self.cfg.gap_left, self.cfg.gap_right
        elif self.l_counter < self.cfg.n_s and self.r_counter >= self.cfg.n_s:
            self.state = "L_UNC"
            zeta_min, zeta_max = -self.cfg.alpha_safe, self.cfg.gap_right
        elif self.l_counter >= self.cfg.n_s and self.r_counter < self.cfg.n_s:
            self.state = "R_UNC"
            zeta_min, zeta_max = self.cfg.gap_left, self.cfg.alpha_safe
        else:
            self.state = "FULL"
            zeta_min, zeta_max = -180.0, 180.0

        return (old_state != self.state), self.state, zeta_min, zeta_max

    def get_stats(self) -> dict:
        """Get current state machine statistics"""
        return {
            "state": self.state,
            "l_counter": self.l_counter,
            "r_counter": self.r_counter,
            "n_s": self.cfg.n_s,
        }


class AdaptiveScannerApp:
    """Main application integrating all components"""

    def __init__(self, config: AdaptiveConfig, verbose: bool = False,
                 enable_sbes: bool = False,
                 msis_address: Optional[str] = None,
                 sbes_left_address: Optional[str] = None,
                 sbes_right_address: Optional[str] = None,
                 msis_configurator_address: Optional[str] = None,
                 msis_recv_timeout_ms: int = 1000,
                 sbes_recv_timeout_ms: int = 100,
                 config_request_timeout_ms: int = 5000):
        self.config = config
        self.verbose = verbose
        self.enable_sbes = enable_sbes
        self.running = True
        self._msis_recv_timeout_ms = msis_recv_timeout_ms
        self._sbes_recv_timeout_ms = sbes_recv_timeout_ms
        self.scanner = StepAdaptiveScanner(config)
        self.subscriber = MSISSubscriber(msis_address)
        self.configurator = MSISConfigurator(msis_configurator_address, config_request_timeout_ms)

        # SBES components (optional)
        self.sbes_subscriber: Optional[SBESSubscriber] = None
        self.scan_range_sm: Optional[ScanRangeStateMachine] = None
        if enable_sbes:
            self.sbes_subscriber = SBESSubscriber(sbes_left_address, sbes_right_address)
            self.scan_range_sm = ScanRangeStateMachine(config)

        # Current scan range (can be modified by SBES state machine)
        self.current_rotation_min = config.rotation_min
        self.current_rotation_max = config.rotation_max

        # Statistics
        self.msg_count = 0
        self.mode_changes = 0
        self.range_changes = 0
        self.start_time: Optional[float] = None

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\nReceived Ctrl+C, shutting down...")
        self.running = False

    def sync_initial_state(self) -> bool:
        """
        Synchronize initial state with simulation.

        This method:
        1. Queries current MSIS configuration from simulation
        2. Sends initial configuration to ensure consistent state
        3. Resets internal scanner state to match

        Returns:
            True if synchronization successful
        """
        print("\n--- Synchronizing initial state with simulation ---")

        # Step 1: Query current configuration from simulation
        try:
            print("Querying current MSIS configuration...")
            current_config = self.configurator.get_config()
            if current_config.get('status') == 'ok':
                sim_rotation_min = current_config.get('rotation_min', self.config.rotation_min)
                sim_rotation_max = current_config.get('rotation_max', self.config.rotation_max)
                sim_step_mult = current_config.get('step_multiplier', self.config.step_multiplier_coarse)
                print(f"  Current sim config: rotation=[{sim_rotation_min}, {sim_rotation_max}], "
                      f"step_multiplier={sim_step_mult}")
            else:
                print(f"  Warning: get_config returned status={current_config.get('status')}")
        except Exception as e:
            print(f"  Warning: Failed to query current config: {e}")

        # Step 2: Send initial configuration (COARSE mode with configured scan range)
        try:
            print(f"Sending initial config: rotation=[{self.current_rotation_min}, {self.current_rotation_max}], "
                  f"step_multiplier={self.config.step_multiplier_coarse}")
            reply = self.configurator.send_config(
                self.current_rotation_min,
                self.current_rotation_max,
                self.config.step_multiplier_coarse
            )
            if reply.get('status') == 'ok':
                print(f"  Config applied successfully")
            else:
                print(f"  Warning: Config reply status={reply.get('status')}")
                return False
        except Exception as e:
            print(f"  Error: Failed to send initial config: {e}")
            return False

        # Step 3: Reset internal scanner state to match
        self.scanner.current_mode = "COARSE"
        self.scanner.current_step_multiplier = self.config.step_multiplier_coarse
        self.scanner.detection_history.clear()
        self.scanner.total_samples = 0
        self.scanner.total_detections = 0

        # Reset SBES state machine if enabled
        if self.scan_range_sm:
            self.scan_range_sm.l_counter = 0
            self.scan_range_sm.r_counter = 0
            self.scan_range_sm.state = "FULL"
            # If SBES is enabled and we start in FULL mode, update rotation range
            if self.enable_sbes:
                # Don't change to [-180, 180] here, keep user-configured range
                # The SBES state machine will update range based on detections
                pass

        print("--- Initial state synchronized ---\n")
        return True

    def send_mode_config(self, step_multiplier: int,
                         rotation_min: Optional[float] = None,
                         rotation_max: Optional[float] = None) -> bool:
        """Send updated configuration (step and/or rotation range)"""
        if rotation_min is None:
            rotation_min = self.current_rotation_min
        if rotation_max is None:
            rotation_max = self.current_rotation_max
        try:
            reply = self.configurator.send_config(
                rotation_min,
                rotation_max,
                step_multiplier
            )
            return reply.get('status') == 'ok'
        except Exception as e:
            print(f"Warning: Failed to send config: {e}")
            return False

    def check_sbes_detection(self, sbes_msg: Optional[dict]) -> bool:
        """Check if SBES message indicates obstacle detection"""
        if sbes_msg is None:
            return False
        # Check 'detected' field first (boolean)
        if 'detected' in sbes_msg:
            return bool(sbes_msg['detected'])
        # Fallback: check distance field
        distance = sbes_msg.get('distance', -1)
        if distance <= 0:
            return False
        return distance < self.config.sbes_max_range

    def format_status(self, msg: dict, detected: bool) -> str:
        """Format status line for output"""
        stats = self.scanner.get_stats()
        ts = datetime.fromtimestamp(msg.get('timestamp', time.time()))
        angle = msg.get('angle', 0.0)
        eta_c = stats['detection_density']

        step_deg = stats['step_multiplier'] * 0.5

        status = (
            f"[{self.msg_count:5d}] {ts.strftime('%H:%M:%S.%f')[:-3]} | "
            f"angle={angle:6.1f} deg | "
            f"det={'Y' if detected else 'N'} | "
            f"eta_c={eta_c:.3f} | "
            f"mode={stats['current_mode']:6s} | "
            f"step={step_deg:.1f} deg | "
            f"range=[{self.current_rotation_min:6.1f}, {self.current_rotation_max:6.1f}]"
        )

        # Add SBES state info if enabled
        if self.enable_sbes and self.scan_range_sm:
            sm_stats = self.scan_range_sm.get_stats()
            status += (
                f" | SBES={sm_stats['state']:5s} "
                f"(L:{sm_stats['l_counter']}/R:{sm_stats['r_counter']})"
            )

        return status

    def run(self) -> None:
        """Main application loop"""
        signal.signal(signal.SIGINT, self.signal_handler)

        print("=" * 80)
        print("MSIS Adaptive Scanner")
        print("=" * 80)
        print(f"Configuration:")
        print(f"  Window duration: {self.config.window_duration}s ({self.config.window_samples} samples)")
        print(f"  Density thresholds: low={self.config.density_low}, high={self.config.density_high}")
        print(f"  Step multipliers: fine={self.config.step_multiplier_fine}, coarse={self.config.step_multiplier_coarse}")
        print(f"  Intensity threshold: {self.config.intensity_threshold}")
        print(f"  Scan range: [{self.config.rotation_min}, {self.config.rotation_max}] deg")
        print(f"  Safety half-angle (alpha_safe): {self.config.alpha_safe} deg")
        print(f"  SBES gap region: [{self.config.gap_left}, {self.config.gap_right}] deg")
        if self.enable_sbes:
            print(f"  SBES enabled: Yes")
            print(f"    Stability threshold (N_S): {self.config.n_s}")
            print(f"    SBES max range: {self.config.sbes_max_range} m")
        else:
            print(f"  SBES enabled: No")
        print("=" * 80)

        # Connect to MSIS subscriber
        print(f"Connecting to MSIS at {self.subscriber.address}...")
        try:
            self.subscriber.connect(recv_timeout_ms=self._msis_recv_timeout_ms)
        except Exception as e:
            print(f"Error: Failed to connect to MSIS: {e}")
            return
        print("MSIS connected.")

        # Connect to SBES subscribers if enabled
        if self.enable_sbes and self.sbes_subscriber:
            print(f"Connecting to SBES (left: {self.sbes_subscriber.left_address}, "
                  f"right: {self.sbes_subscriber.right_address})...")
            try:
                self.sbes_subscriber.connect(recv_timeout_ms=self._sbes_recv_timeout_ms)
                print("SBES connected.")
            except Exception as e:
                print(f"Warning: Failed to connect to SBES: {e}")
                print("Continuing without SBES scan range switching...")
                self.enable_sbes = False
                self.sbes_subscriber = None
                self.scan_range_sm = None

        print("Waiting for messages...")

        # Synchronize initial state with simulation
        if not self.sync_initial_state():
            print("Warning: Initial state synchronization failed, continuing anyway...")

        print("Press Ctrl+C to stop\n")
        self.start_time = time.time()

        try:
            while self.running:
                # Process SBES messages if enabled (non-blocking)
                range_changed = False
                if self.enable_sbes and self.sbes_subscriber and self.scan_range_sm:
                    left_msg = self.sbes_subscriber.receive_left()
                    right_msg = self.sbes_subscriber.receive_right()

                    left_detect = self.check_sbes_detection(left_msg)
                    right_detect = self.check_sbes_detection(right_msg)

                    # Update scan range state machine
                    range_changed, range_state, zeta_min, zeta_max = \
                        self.scan_range_sm.update(left_detect, right_detect)

                    if range_changed:
                        self.range_changes += 1
                        self.current_rotation_min = zeta_min
                        self.current_rotation_max = zeta_max
                        print(f"\n>>> RANGE CHANGE #{self.range_changes}: -> {range_state} "
                              f"(rotation=[{zeta_min:.1f}, {zeta_max:.1f}] deg)")

                # Receive MSIS message
                msg = self.subscriber.receive()
                if msg is None:
                    continue  # Timeout, check running flag

                self.msg_count += 1

                # Extract and process beam_data
                beam_data = msg.get('beam_data')
                detected = process_beam_data(beam_data, self.config.intensity_threshold)

                # Add to sliding window
                self.scanner.add_detection(detected)

                # Check for step mode change
                mode_changed, mode, step_mult = self.scanner.update_scan_mode()

                # Send new configuration if mode or range changed
                if mode_changed or range_changed:
                    if mode_changed:
                        self.mode_changes += 1
                        print(f"\n>>> MODE CHANGE #{self.mode_changes}: -> {mode} (step_multiplier={step_mult})")

                    if self.send_mode_config(step_mult):
                        print(f">>> Configuration updated successfully\n")
                    else:
                        print(f">>> WARNING: Configuration update failed\n")

                # Output status
                if self.verbose or mode_changed or range_changed:
                    status = self.format_status(msg, detected)
                    print(status)
                elif self.msg_count % 10 == 0:
                    # Print every 10th message in non-verbose mode
                    status = self.format_status(msg, detected)
                    print(f"\r{status}", end='', flush=True)

        finally:
            self.subscriber.close()
            if self.sbes_subscriber:
                self.sbes_subscriber.close()
            self.print_summary()

    def print_summary(self) -> None:
        """Print session summary"""
        duration = time.time() - self.start_time if self.start_time else 0
        stats = self.scanner.get_stats()

        print("\n" + "=" * 80)
        print("Session Summary")
        print("=" * 80)
        print(f"Duration: {duration:.1f}s")
        print(f"Messages received: {self.msg_count}")
        print(f"Total detections: {stats['total_detections']}")
        print(f"Overall detection rate: {stats['total_detections']/max(stats['total_samples'],1):.3f}")
        print(f"Step mode changes: {self.mode_changes}")
        print(f"Final step mode: {stats['current_mode']}")
        if self.enable_sbes and self.scan_range_sm:
            sm_stats = self.scan_range_sm.get_stats()
            print(f"Range changes: {self.range_changes}")
            print(f"Final range state: {sm_stats['state']}")
            print(f"Final rotation: [{self.current_rotation_min:.1f}, {self.current_rotation_max:.1f}] deg")
        print("=" * 80)


def parse_args(defaults: dict = None):
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description='MSIS Adaptive Scanner - Step angle adaptive control based on detection density',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument('--threshold', type=int, default=60,
                        help='Detection intensity threshold (0-255)')
    parser.add_argument('--window', type=float, default=2.0,
                        help='Sliding window duration in seconds')
    parser.add_argument('--rotation-min', type=float, default=-45.0,
                        help='Minimum scan angle (degrees)')
    parser.add_argument('--rotation-max', type=float, default=45.0,
                        help='Maximum scan angle (degrees)')
    parser.add_argument('--step-fine', type=int, default=1,
                        help='Fine mode step multiplier')
    parser.add_argument('--step-coarse', type=int, default=3,
                        help='Coarse mode step multiplier')
    parser.add_argument('--density-high', type=float, default=0.6,
                        help='High density threshold for fine scan')
    parser.add_argument('--density-low', type=float, default=0.4,
                        help='Low density threshold for coarse scan')
    parser.add_argument('--sample-rate', type=float, default=10.0,
                        help='Expected MSIS sample rate (Hz)')
    parser.add_argument('--alpha-safe', type=float, default=28.0,
                        help='Safety half-angle (degrees) for partial SBES coverage')
    parser.add_argument('--gap-left', type=float, default=-62.5,
                        help='Left boundary of SBES coverage gap (degrees)')
    parser.add_argument('--gap-right', type=float, default=62.5,
                        help='Right boundary of SBES coverage gap (degrees)')
    parser.add_argument('--n-s', type=int, default=3,
                        help='Stability threshold for SBES state machine (consecutive detection count)')
    parser.add_argument('--sbes-max-range', type=float, default=100.0,
                        help='SBES max detection range (meters)')
    parser.add_argument('--enable-sbes', action='store_true',
                        help='Enable SBES-based scan range switching')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output (print every message)')

    if defaults:
        parser.set_defaults(**defaults)
    return parser.parse_args()


def main():
    cfg_path = os.path.join(_SCRIPT_DIR, '..', 'config', 'msis_adaptive_scanner.jsonc')
    try:
        file_cfg = load_jsonc(cfg_path)
    except Exception as e:
        print(f"Warning: Failed to load config {cfg_path}: {e}")
        file_cfg = {}

    net   = file_cfg.get('network', {})
    to_ms = file_cfg.get('timeouts_ms', {})
    sw    = file_cfg.get('sliding_window', {})
    sa    = file_cfg.get('step_adaptation', {})
    det   = file_cfg.get('detection', {})
    sr    = file_cfg.get('scan_range', {})
    sbes  = file_cfg.get('sbes', {})
    rt    = file_cfg.get('runtime', {})

    arg_defaults = {}
    # sliding_window
    if 'window_duration'        in sw:  arg_defaults['window']        = sw['window_duration']
    if 'sample_rate'            in sw:  arg_defaults['sample_rate']   = sw['sample_rate']
    # step_adaptation
    if 'density_high'           in sa:  arg_defaults['density_high']  = sa['density_high']
    if 'density_low'            in sa:  arg_defaults['density_low']   = sa['density_low']
    if 'step_multiplier_fine'   in sa:  arg_defaults['step_fine']     = sa['step_multiplier_fine']
    if 'step_multiplier_coarse' in sa:  arg_defaults['step_coarse']   = sa['step_multiplier_coarse']
    # detection
    if 'intensity_threshold'    in det: arg_defaults['threshold']     = det['intensity_threshold']
    # scan_range
    if 'rotation_min'           in sr:  arg_defaults['rotation_min']  = sr['rotation_min']
    if 'rotation_max'           in sr:  arg_defaults['rotation_max']  = sr['rotation_max']
    # sbes
    if 'alpha_safe'             in sbes: arg_defaults['alpha_safe']   = sbes['alpha_safe']
    if 'gap_left'               in sbes: arg_defaults['gap_left']     = sbes['gap_left']
    if 'gap_right'              in sbes: arg_defaults['gap_right']    = sbes['gap_right']
    if 'n_s'                    in sbes: arg_defaults['n_s']          = sbes['n_s']
    if 'sbes_max_range'         in sbes: arg_defaults['sbes_max_range'] = sbes['sbes_max_range']
    # runtime
    if 'enable_sbes'            in rt:  arg_defaults['enable_sbes']   = rt['enable_sbes']
    if 'verbose'                in rt:  arg_defaults['verbose']       = rt['verbose']

    args = parse_args(arg_defaults)

    # Build configuration from arguments
    config = AdaptiveConfig(
        window_duration=args.window,
        sample_rate=args.sample_rate,
        density_high=args.density_high,
        density_low=args.density_low,
        step_multiplier_fine=args.step_fine,
        step_multiplier_coarse=args.step_coarse,
        intensity_threshold=args.threshold,
        rotation_min=args.rotation_min,
        rotation_max=args.rotation_max,
        alpha_safe=args.alpha_safe,
        gap_left=args.gap_left,
        gap_right=args.gap_right,
        n_s=args.n_s,
        sbes_max_range=args.sbes_max_range,
    )

    # Create and run application
    app = AdaptiveScannerApp(
        config,
        verbose=args.verbose,
        enable_sbes=args.enable_sbes,
        msis_address=net.get('msis_subscriber_address'),
        sbes_left_address=net.get('sbes_left_address'),
        sbes_right_address=net.get('sbes_right_address'),
        msis_configurator_address=net.get('msis_configurator_address'),
        msis_recv_timeout_ms=to_ms.get('msis_recv', 1000),
        sbes_recv_timeout_ms=to_ms.get('sbes_recv', 100),
        config_request_timeout_ms=to_ms.get('config_request', 5000),
    )
    app.run()


if __name__ == '__main__':
    main()
