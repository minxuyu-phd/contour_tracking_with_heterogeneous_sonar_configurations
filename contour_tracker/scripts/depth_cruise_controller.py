#!/usr/bin/env python3
"""
Depth Cruise Controller for SAM-AUV

Integrated controller for stable cruise at a set depth, speed, and heading.
Uses two-mode control strategy:
- DIVE mode: Descent from surface using VBS + LCG + low-speed RPM
- CRUISE mode: Depth controlled via pitch angle (zero buoyancy + rudder control)

Performance tuning: 11m, 1.2m/s

=============================================================================
Usage:
    python depth_cruise_controller.py
    python depth_cruise_controller.py --config path/to/depth_cruise_controller.jsonc
"""

import argparse
import csv
import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Dict, Any

try:
    from pid_base import PIDController, AnglePIDController, PIDParams
except ImportError:
    from .pid_base import PIDController, AnglePIDController, PIDParams

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from nng_comm.scripts import Subscriber, Replier, Publisher

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def load_jsonc(path: str) -> dict:
    """Load a JSONC file (JSON with // comments, correctly skips // inside strings)"""
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


class SensorSubscriber:
    """Wrapper around nng_comm.Subscriber that stores latest data with optional extraction."""

    def __init__(self, address: str, data_extractor=None):
        self.address = address
        self.data_extractor = data_extractor
        self._sub = Subscriber()
        self._latest_data = None
        self._lock = threading.Lock()
        self._recv_count = 0

    def _on_message(self, msg: dict):
        with self._lock:
            self._recv_count += 1
            if self.data_extractor:
                self._latest_data = self.data_extractor(msg)
            else:
                self._latest_data = msg

    def start(self):
        if not self._sub.init(self.address):
            print(f"SensorSubscriber init failed: {self.address}")
            return
        self._sub.set_callback(self._on_message)
        self._sub.start_async()

    def stop(self):
        self._sub.close()

    @property
    def latest_data(self):
        with self._lock:
            return self._latest_data

    @property
    def is_connected(self) -> bool:
        return self._recv_count > 0


class ActuatorPublisher:
    """Wrapper around nng_comm.Publisher that dials to an address and adds timestamps."""

    def __init__(self, address: str):
        self.address = address
        self._pub = Publisher()

    def connect(self):
        self._pub.init(self.address)
        time.sleep(0.1)  # Allow connection to establish

    def send(self, data: dict):
        if 'timestamp' not in data:
            data['timestamp'] = time.time()
        self._pub.publish(data)

    def close(self):
        self._pub.close()


@dataclass
class OdometryData:
    """Container for odometry sensor data"""
    # Position (NED)
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0  # depth (positive down)
    # Velocity
    vel_x: float = 0.0  # forward speed
    vel_y: float = 0.0
    vel_z: float = 0.0
    # Orientation
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0  # heading
    timestamp: float = 0.0

    # Convenience properties (for backward compatibility)
    @property
    def depth(self) -> float:
        return self.pos_z

    @property
    def speed(self) -> float:
        return self.vel_x

    @property
    def heading(self) -> float:
        return self.yaw


class DepthCruiseController:
    """
    Depth Cruise Controller - Integrated depth, speed, and heading control

    Control Modes (Two-Mode Design):
    - DIVE Mode: Initial descent from surface
      - VBS: negative buoyancy (70-80%) for sinking
      - LCG: rear (70%) for nose-down attitude
      - RPM: low speed (target ~0.3 m/s) for gentle forward motion
      - Pitch: mild nose-down (-5° to -10°)

    - CRUISE Mode: Stable cruise at target depth
      - VBS: neutral buoyancy (50%) for zero net force
      - LCG: center (50%) or assist pitch control
      - RPM: normal speed (controlled by speed PID)
      - Pitch: small angles (±3-5°) via rudder for depth control

    Mode Transitions:
    - DIVE -> CRUISE: when depth >= DIVE_DEPTH_THRESHOLD (2.0m)
    - CRUISE -> DIVE: when depth < DIVE_DEPTH_THRESHOLD - HYSTERESIS (1.5m) AND not reached target
    """

    # Control modes (simplified from three to two)
    MODE_DIVE = "dive"      # Descent phase
    MODE_CRUISE = "cruise"  # Cruise phase (renamed from MODE_PITCH)

    # Dynamic mode states
    STATE_IDLE = "idle"       # Waiting for setpoint
    STATE_ACTIVE = "active"   # Control running
    STATE_PAUSED = "paused"   # Control paused after stop_control

    def __init__(self, config_path: str):
        """
        Initialize depth cruise controller

        Args:
            config_path: Path to JSONC configuration file
        """
        cfg = load_jsonc(config_path)

        # --- Network ---
        net = cfg.get("network", {})
        self.SERVICE_ADDR = net.get("service_addr", "tcp://192.168.5.10:7780")
        self.ODOMETRY_ADDR = net.get("odometry_addr", "tcp://192.168.5.11:5555")
        self.THRUSTER1_ADDR = net.get("thruster1_addr", "tcp://192.168.5.10:6557")
        self.THRUSTER2_ADDR = net.get("thruster2_addr", "tcp://192.168.5.10:6558")
        self.THRUST_ANGLE_ADDR = net.get("thrust_angle_addr", "tcp://192.168.5.10:6559")
        self.VBS_ADDR = net.get("vbs_addr", "tcp://192.168.5.10:6555")
        self.LCG_ADDR = net.get("lcg_addr", "tcp://192.168.5.10:6556")

        # --- Runtime ---
        rt = cfg.get("runtime", {})
        control_rate = rt.get("control_rate", 30.0)
        self._logging_enabled = rt.get("enable_logging", False)

        # --- Dive mode ---
        dive = cfg.get("dive_mode", {})
        self.DIVE_DEPTH_THRESHOLD = dive.get("depth_threshold", 2.0)
        self.DIVE_DEPTH_HYSTERESIS = dive.get("depth_hysteresis", 0.5)
        self.DIVE_VBS_SETPOINT = dive.get("vbs_setpoint", 75.0)
        self.DIVE_LCG_SETPOINT = dive.get("lcg_setpoint", 70.0)
        self.DIVE_PITCH_TARGET = dive.get("pitch_target", -0.10)
        self.DIVE_SPEED_TARGET = dive.get("speed_target", 0.3)

        # --- Cruise mode ---
        cruise = cfg.get("cruise_mode", {})
        self.CRUISE_VBS_NEUTRAL = cruise.get("vbs_neutral", 50.0)
        self.CRUISE_LCG_CENTER = cruise.get("lcg_center", 50.0)

        # --- Actuator limits ---
        al = cfg.get("actuator_limits", {})
        self.RPM_MIN = al.get("rpm_min", -1000.0)
        self.RPM_MAX = al.get("rpm_max", 1000.0)
        self.RUDDER_MIN = al.get("rudder_min", -0.12)
        self.RUDDER_MAX = al.get("rudder_max", 0.12)
        self.VBS_MIN = al.get("vbs_min", 0.0)
        self.VBS_MAX = al.get("vbs_max", 100.0)
        self.LCG_MIN = al.get("lcg_min", 0.0)
        self.LCG_MAX = al.get("lcg_max", 100.0)
        self.LCG_CENTER = al.get("lcg_center", 50.0)
        self.ELEVATOR_MIN = al.get("elevator_min", -0.12)
        self.ELEVATOR_MAX = al.get("elevator_max", 0.12)

        # --- Rate limits ---
        rl = cfg.get("rate_limits", {})
        self.ELEVATOR_RATE_LIMIT = rl.get("elevator", 0.05)
        self.RUDDER_RATE_LIMIT = rl.get("rudder", 0.01)
        self.HEADING_RATE_LIMIT = rl.get("heading", 0.02)

        # --- PID: Speed ---
        ps = cfg.get("pid_speed", {})
        self.SPEED_KP = ps.get("kp", 500.0)
        self.SPEED_KI = ps.get("ki", 100.0)
        self.SPEED_KD = ps.get("kd", 50.0)
        self.SPEED_INTEGRAL_MAX = ps.get("integral_max", 500.0)

        # --- PID: Heading ---
        ph = cfg.get("pid_heading", {})
        self.HEADING_KP = ph.get("kp", 1.0)
        self.HEADING_KI = ph.get("ki", 0.1)
        self.HEADING_KD = ph.get("kd", 0.2)
        self.HEADING_INTEGRAL_MAX = ph.get("integral_max", 0.5)

        # --- PID: Depth-pitch ---
        pdp = cfg.get("pid_depth_pitch", {})
        self.DEPTH_PITCH_KP = pdp.get("kp", 0.10)
        self.DEPTH_PITCH_KI = pdp.get("ki", 0.015)
        self.DEPTH_PITCH_KD = pdp.get("kd", 0.30)
        self.DEPTH_PITCH_INTEGRAL_MAX = pdp.get("integral_max", 0.06)

        # --- PID: Pitch ---
        pp = cfg.get("pid_pitch", {})
        self.PITCH_KP = pp.get("kp", 0.75)
        self.PITCH_KI = pp.get("ki", 0.12)
        self.PITCH_KD = pp.get("kd", 0.32)
        self.PITCH_INTEGRAL_MAX = pp.get("integral_max", 0.35)

        # --- PID: Roll ---
        pr = cfg.get("pid_roll", {})
        self.ROLL_STABILIZATION_ENABLED = pr.get("enabled", True)
        self.ROLL_KP = pr.get("kp", 50.0)
        self.ROLL_KI = pr.get("ki", 0.0)
        self.ROLL_KD = pr.get("kd", 30.0)
        self.ROLL_INTEGRAL_MAX = pr.get("integral_max", 20.0)
        self.ROLL_DIFF_MAX = pr.get("diff_max", 80.0)
        self.ROLL_DEADZONE = pr.get("deadzone", 0.02)

        # --- Pitch limits ---
        pl = cfg.get("pitch_limits", {})
        self.PITCH_LIMIT_DESCENT = pl.get("descent", 0.35)
        self.PITCH_LIMIT_STEADY = pl.get("steady", 0.10)
        self.PITCH_LIMIT = pl.get("default", 0.35)

        # --- Depth steady ---
        ds = cfg.get("depth_steady", {})
        self.DEPTH_STEADY_THRESHOLD = ds.get("threshold", 0.5)
        self.DEPTH_STEADY_HYSTERESIS = ds.get("hysteresis", 1.0)

        # --- Pitch feedforward ---
        pf = cfg.get("pitch_feedforward", {})
        self.PITCH_FEEDFORWARD_PER_SPEED_DESCENT = pf.get("per_speed_descent", -0.08)
        self.PITCH_FEEDFORWARD_PER_SPEED_STEADY = pf.get("per_speed_steady", -0.08)
        self.PITCH_FEEDFORWARD_PER_SPEED = pf.get("per_speed_default", -0.08)

        # --- Sensor limits ---
        sl = cfg.get("sensor_limits", {})
        self.SENSOR_DEPTH_MAX = sl.get("depth_max", 10000.0)
        self.SENSOR_SPEED_MAX = sl.get("speed_max", 1000.0)
        self.SENSOR_RATE_MAX_DEPTH = sl.get("rate_max_depth", 1000.0)
        self.SENSOR_RATE_MAX_SPEED = sl.get("rate_max_speed", 10000.0)

        # --- Sensor EMA filter ---
        sf_filter = cfg.get("sensor_filter", {})
        self.FILTER_DEPTH_ALPHA = sf_filter.get("depth_alpha", 0.4)
        self.FILTER_SPEED_ALPHA = sf_filter.get("speed_alpha", 0.3)
        self.FILTER_HEADING_ALPHA = sf_filter.get("heading_alpha", 0.8)
        self.FILTER_PITCH_ALPHA = sf_filter.get("pitch_alpha", 0.5)
        self.FILTER_ROLL_ALPHA = sf_filter.get("roll_alpha", 0.6)

        # --- Safety ---
        sf = cfg.get("safety", {})
        self.DANGEROUS_VEL_Y_THRESHOLD = sf.get("dangerous_vel_y_threshold", 1.0)
        self.DANGEROUS_ROLL_RATE_THRESHOLD = sf.get("dangerous_roll_rate_threshold", 0.5)
        self.DANGEROUS_STATE_CONTROL_FACTOR = sf.get("dangerous_state_control_factor", 0.3)

        # --- LCG pitch ratio ---
        self.LCG_PITCH_RATIO = cfg.get("lcg_pitch_ratio", 0.3)

        # Setpoints (will be set via service)
        self.target_depth = 0.0
        self.target_speed = 0.0
        self.target_heading = 0.0
        self._smoothed_heading = 0.0  # Smoothed heading setpoint for rate limiting

        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        self.vbs_neutral = self.CRUISE_VBS_NEUTRAL

        # Dynamic mode support
        self._service_addr = self.SERVICE_ADDR
        self._state = self.STATE_IDLE
        self._state_lock = threading.RLock()
        self._control_enabled = threading.Event()
        self._shutdown_event = threading.Event()
        self._service: Optional[Replier] = None
        self._control_thread: Optional[threading.Thread] = None

        # Current control mode - start in DIVE mode
        self._current_mode = self.MODE_DIVE

        # Running flag
        self._running = False

        # Sensor data
        self._odom_data = OdometryData()

        # Filtered sensor values (initialized on first valid reading)
        self._filtered_depth = None
        self._filtered_speed = None
        self._filtered_heading = None
        self._filtered_pitch = None
        self._filtered_roll = None

        # Subscribers and publishers
        self._odom_subscriber: Optional[SensorSubscriber] = None
        self._thruster1_pub: Optional[ActuatorPublisher] = None
        self._thruster2_pub: Optional[ActuatorPublisher] = None
        self._angle_pub: Optional[ActuatorPublisher] = None
        self._vbs_pub: Optional[ActuatorPublisher] = None
        self._lcg_pub: Optional[ActuatorPublisher] = None

        # PID controllers
        self._speed_pid: Optional[PIDController] = None
        self._heading_pid: Optional[AnglePIDController] = None
        self._depth_pitch_pid: Optional[PIDController] = None
        self._pitch_pid: Optional[AnglePIDController] = None
        self._roll_pid: Optional[AnglePIDController] = None

        # Last outputs for recording
        self._last_rpm = 0.0
        self._last_rpm1 = 0.0  # Left thruster
        self._last_rpm2 = 0.0  # Right thruster
        self._last_diff_rpm = 0.0  # Differential RPM for roll stabilization
        self._last_rudder = 0.0
        self._last_elevator = 0.0  # vertical thrust angle
        self._last_vbs = self.vbs_neutral
        self._last_lcg = self.LCG_CENTER
        self._last_pitch_sp = 0.0

        # Data recording
        self._records = []
        self._start_time = None

        # Mode transition tracking
        self._reached_target_depth = False
        self._in_steady_phase = False
        self._vbs_integral = 0.0
        self._transition_frames = 0

        # Signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        """Handle shutdown signals"""
        print("\nReceived shutdown signal, stopping...")
        self._running = False
        self._shutdown_event.set()
        self._control_enabled.set()

    def _setup_pid_controllers(self):
        """Initialize all PID controllers"""
        # Speed PID
        self._speed_pid = PIDController(PIDParams(
            kp=self.SPEED_KP,
            ki=self.SPEED_KI,
            kd=self.SPEED_KD,
            output_min=self.RPM_MIN,
            output_max=self.RPM_MAX,
            integral_max=self.SPEED_INTEGRAL_MAX
        ))

        # Heading PID
        self._heading_pid = AnglePIDController(PIDParams(
            kp=self.HEADING_KP,
            ki=self.HEADING_KI,
            kd=self.HEADING_KD,
            output_min=self.RUDDER_MIN,
            output_max=self.RUDDER_MAX,
            integral_max=self.HEADING_INTEGRAL_MAX
        ))

        # Depth PID (Cruise mode) - outputs target pitch angle
        self._depth_pitch_pid = PIDController(PIDParams(
            kp=self.DEPTH_PITCH_KP,
            ki=self.DEPTH_PITCH_KI,
            kd=self.DEPTH_PITCH_KD,
            output_min=-self.PITCH_LIMIT,
            output_max=self.PITCH_LIMIT,
            integral_max=self.DEPTH_PITCH_INTEGRAL_MAX
        ))

        # Pitch PID (inner loop) - outputs thrust vector vertical angle (elevator)
        self._pitch_pid = AnglePIDController(PIDParams(
            kp=self.PITCH_KP,
            ki=self.PITCH_KI,
            kd=self.PITCH_KD,
            output_min=self.ELEVATOR_MIN,
            output_max=self.ELEVATOR_MAX,
            integral_max=self.PITCH_INTEGRAL_MAX
        ))

        # Roll PID - outputs differential RPM for roll stabilization
        self._roll_pid = AnglePIDController(PIDParams(
            kp=self.ROLL_KP,
            ki=self.ROLL_KI,
            kd=self.ROLL_KD,
            output_min=-self.ROLL_DIFF_MAX,
            output_max=self.ROLL_DIFF_MAX,
            integral_max=self.ROLL_INTEGRAL_MAX
        ))

    def _setup_subscribers(self):
        """Setup sensor subscribers"""
        def extract_odom(msg):
            """Extract complete data from odometry message"""
            data = OdometryData()
            data.timestamp = msg.get('timestamp', time.time())

            # Position (NED)
            if 'position' in msg:
                data.pos_x = msg['position'].get('x', 0.0)
                data.pos_y = msg['position'].get('y', 0.0)
                data.pos_z = msg['position'].get('z', 0.0)

            # Velocity
            if 'linear_velocity' in msg:
                data.vel_x = msg['linear_velocity'].get('x', 0.0)
                data.vel_y = msg['linear_velocity'].get('y', 0.0)
                data.vel_z = msg['linear_velocity'].get('z', 0.0)

            # Orientation
            if 'orientation' in msg:
                data.roll = msg['orientation'].get('roll', 0.0)
                data.pitch = msg['orientation'].get('pitch', 0.0)
                data.yaw = msg['orientation'].get('yaw', 0.0)

            return data

        self._odom_subscriber = SensorSubscriber(
            self.ODOMETRY_ADDR,
            data_extractor=extract_odom
        )

    def _setup_publishers(self):
        """Setup actuator publishers"""
        self._thruster1_pub = ActuatorPublisher(self.THRUSTER1_ADDR)
        self._thruster2_pub = ActuatorPublisher(self.THRUSTER2_ADDR)
        self._angle_pub = ActuatorPublisher(self.THRUST_ANGLE_ADDR)
        self._vbs_pub = ActuatorPublisher(self.VBS_ADDR)
        self._lcg_pub = ActuatorPublisher(self.LCG_ADDR)

    def _send_thruster_command(self, rpm: float, diff_rpm: float = 0.0):
        """Send RPM command to thrusters with optional differential for roll control"""
        rpm1 = rpm - diff_rpm  # Left thruster
        rpm2 = rpm + diff_rpm  # Right thruster

        # Clamp to limits
        rpm1 = max(self.RPM_MIN, min(self.RPM_MAX, rpm1))
        rpm2 = max(self.RPM_MIN, min(self.RPM_MAX, rpm2))

        self._last_rpm = rpm
        self._last_rpm1 = rpm1
        self._last_rpm2 = rpm2
        self._last_diff_rpm = diff_rpm

        cmd1 = {"timestamp": time.time(), "rpm": rpm1}
        cmd2 = {"timestamp": time.time(), "rpm": rpm2}
        self._thruster1_pub.send(cmd1)
        self._thruster2_pub.send(cmd2)

    def _send_thrust_angle_command(self, horizontal: float, vertical: float = 0.0):
        """Send thrust angle command for heading and pitch control"""
        horizontal = max(self.RUDDER_MIN, min(self.RUDDER_MAX, horizontal))
        vertical = max(self.ELEVATOR_MIN, min(self.ELEVATOR_MAX, vertical))

        # Apply rate limiting to rudder
        rudder_delta = horizontal - self._last_rudder
        if abs(rudder_delta) > self.RUDDER_RATE_LIMIT:
            horizontal = self._last_rudder + self.RUDDER_RATE_LIMIT * (1 if rudder_delta > 0 else -1)

        # Apply rate limiting to elevator
        elevator_delta = vertical - self._last_elevator
        if abs(elevator_delta) > self.ELEVATOR_RATE_LIMIT:
            vertical = self._last_elevator + self.ELEVATOR_RATE_LIMIT * (1 if elevator_delta > 0 else -1)

        self._last_rudder = horizontal
        self._last_elevator = vertical

        cmd = {
            "timestamp": time.time(),
            "horizontal_radians": -horizontal,  # Inverted: positive PID output -> turn right
            "vertical_radians": -vertical  # Inverted for level flight
        }
        self._angle_pub.send(cmd)

    def _send_vbs_command(self, percentage: float):
        """Send VBS command"""
        percentage = max(self.VBS_MIN, min(self.VBS_MAX, percentage))
        self._last_vbs = percentage
        cmd = {"timestamp": time.time(), "percentage": percentage}
        self._vbs_pub.send(cmd)

    def _send_lcg_command(self, percentage: float):
        """Send LCG command"""
        percentage = max(self.LCG_MIN, min(self.LCG_MAX, percentage))
        self._last_lcg = percentage
        cmd = {"timestamp": time.time(), "percentage": percentage}
        self._lcg_pub.send(cmd)

    def _determine_mode(self, current_depth: float) -> str:
        """
        Determine control mode based on depth only (simplified from three-mode)

        Mode transitions:
        - DIVE -> CRUISE: when depth >= DIVE_DEPTH_THRESHOLD
        - CRUISE -> DIVE: when depth < DIVE_DEPTH_THRESHOLD - HYSTERESIS AND not reached target
        """
        # Check if we've reached target depth (sticky flag)
        if abs(self.target_depth - current_depth) < self.DEPTH_STEADY_THRESHOLD:
            self._reached_target_depth = True

        if self._current_mode == self.MODE_DIVE:
            # In DIVE mode, exit when depth threshold reached
            if current_depth >= self.DIVE_DEPTH_THRESHOLD:
                return self.MODE_CRUISE
            return self.MODE_DIVE
        else:
            # In CRUISE mode, return to DIVE only if:
            # 1. Depth dropped below threshold with hysteresis
            # 2. Haven't reached target depth yet
            if current_depth < self.DIVE_DEPTH_THRESHOLD - self.DIVE_DEPTH_HYSTERESIS:
                if not self._reached_target_depth:
                    return self.MODE_DIVE
            return self.MODE_CRUISE

    def _on_mode_switch(self, old_mode: str, new_mode: str):
        """Handle mode transition - reset PIDs for smooth transition"""
        print(f"Mode switch: {old_mode} -> {new_mode}")

        # Reset all PID integrators
        self._speed_pid.reset()
        self._pitch_pid.reset()
        self._depth_pitch_pid.reset()
        self._heading_pid.reset()

        # Reset VBS integral
        self._vbs_integral = 0.0

        # Reset steady phase tracking
        self._in_steady_phase = False

        # Set transition frames for gradual change
        self._transition_frames = 10

    def _record_data(self, elapsed: float, depth_err: float, speed_err: float, heading_err: float,
                     extra_data: dict = None, dt: float = 0.0, count: int = 0):
        """Record control data for later analysis"""
        if not self._logging_enabled:
            return
        odom = self._odom_data
        record = {
            'timestamp': time.time(),
            'elapsed': elapsed,
            'mode': self._current_mode,
            # Setpoints
            'depth_sp': self.target_depth,
            'speed_sp': self.target_speed,
            'heading_sp': self.target_heading,
            'pitch_sp': self._last_pitch_sp,
            # Sensor feedback
            'depth': odom.depth,
            'speed': odom.speed,
            'heading': odom.heading,
            'pitch': odom.pitch,
            'odom_timestamp': odom.timestamp,
            # Raw odometry - position
            'odom_pos_x': odom.pos_x,
            'odom_pos_y': odom.pos_y,
            'odom_pos_z': odom.pos_z,
            # Raw odometry - velocity
            'odom_vel_x': odom.vel_x,
            'odom_vel_y': odom.vel_y,
            'odom_vel_z': odom.vel_z,
            # Raw odometry - orientation
            'odom_roll': odom.roll,
            'odom_pitch': odom.pitch,
            'odom_yaw': odom.yaw,
            # Errors
            'depth_err': depth_err,
            'speed_err': speed_err,
            'heading_err': heading_err,
            # Degrees versions
            'heading_sp_deg': math.degrees(self.target_heading),
            'heading_deg': math.degrees(odom.heading),
            'heading_err_deg': math.degrees(heading_err),
            # Actuator commands
            'cmd_rpm': self._last_rpm,
            'cmd_rudder': self._last_rudder,
            'cmd_elevator': self._last_elevator,
            'cmd_vbs_pct': self._last_vbs,
            'cmd_lcg_pct': self._last_lcg,
            'cmd_rudder_deg': math.degrees(self._last_rudder),
            'cmd_elevator_deg': math.degrees(self._last_elevator),
            # PID internals
            'speed_pid_integral': self._speed_pid.integral if self._speed_pid else 0.0,
            'heading_pid_integral': self._heading_pid.integral if self._heading_pid else 0.0,
            'depth_pitch_pid_integral': self._depth_pitch_pid.integral if self._depth_pitch_pid else 0.0,
            'pitch_pid_integral': self._pitch_pid.integral if self._pitch_pid else 0.0,
            # Filtered sensor values
            'filtered_depth': self._filtered_depth,
            'filtered_speed': self._filtered_speed,
            'filtered_heading': self._filtered_heading,
            'filtered_pitch': self._filtered_pitch,
            'filtered_roll': self._filtered_roll,
            # Control loop diagnostics
            'control_dt': dt,
            'loop_count': count,
        }

        if extra_data:
            record.update(extra_data)

        self._records.append(record)

    def _save_data(self) -> Optional[str]:
        """Save recorded data to CSV file"""
        if not self._logging_enabled or not self._records:
            return None

        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
        os.makedirs(data_dir, exist_ok=True)

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"depth_cruise_{timestamp_str}.csv"
        filepath = os.path.join(data_dir, filename)

        if self._records:
            fieldnames = list(self._records[0].keys())
        else:
            fieldnames = []

        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(self._records)

        return filepath

    def _send_zero_commands(self):
        """Send neutral/zero commands to all actuators"""
        cmd_rpm = {"timestamp": time.time(), "rpm": 0.0}
        self._thruster1_pub.send(cmd_rpm)
        self._thruster2_pub.send(cmd_rpm)

        cmd_angle = {
            "timestamp": time.time(),
            "horizontal_radians": 0.0,
            "vertical_radians": 0.0
        }
        self._angle_pub.send(cmd_angle)

        cmd_vbs = {"timestamp": time.time(), "percentage": 50.0}
        self._vbs_pub.send(cmd_vbs)

        cmd_lcg = {"timestamp": time.time(), "percentage": 50.0}
        self._lcg_pub.send(cmd_lcg)

    # ========== Service Request Handlers ==========

    def _handle_request(self, request: dict) -> dict:
        """Handle service requests"""
        command = request.get("command", "")
        timestamp = time.time()

        if command == "set_setpoint":
            return self._cmd_set_setpoint(request, timestamp)
        elif command == "stop_control":
            return self._cmd_stop_control(timestamp)
        elif command == "get_status":
            return self._cmd_get_status(timestamp)
        elif command == "shutdown":
            return self._cmd_shutdown(timestamp)
        else:
            return {
                "status": "error",
                "command": command,
                "message": f"unknown command: {command}",
                "timestamp": timestamp
            }

    def _cmd_set_setpoint(self, request: dict, timestamp: float) -> dict:
        """Set control setpoints and start/resume control"""
        with self._state_lock:
            # Update setpoints
            if "depth" in request:
                depth = request["depth"]
                if not isinstance(depth, (int, float)) or depth < 0:
                    return {
                        "status": "error",
                        "command": "set_setpoint",
                        "message": "depth must be a non-negative number",
                        "timestamp": timestamp
                    }
                self.target_depth = float(depth)

            if "speed" in request:
                speed = request["speed"]
                if not isinstance(speed, (int, float)):
                    return {
                        "status": "error",
                        "command": "set_setpoint",
                        "message": "speed must be a number",
                        "timestamp": timestamp
                    }
                self.target_speed = float(speed)

            if "heading" in request:
                heading = request["heading"]
                if not isinstance(heading, (int, float)):
                    return {
                        "status": "error",
                        "command": "set_setpoint",
                        "message": "heading must be a number",
                        "timestamp": timestamp
                    }
                if request.get("heading_degrees", True):
                    heading = math.radians(heading)
                self.target_heading = float(heading)

            # Reset state when transitioning from non-active
            if self._state != self.STATE_ACTIVE:
                self._reset_pid_controllers()
                self._current_mode = self.MODE_DIVE
                self._reached_target_depth = False
                self._in_steady_phase = False
                self._vbs_integral = 0.0
                self._smoothed_heading = self.target_heading  # Initialize smoothed heading
                self._filtered_depth = None
                self._filtered_speed = None
                self._filtered_heading = None
                self._filtered_pitch = None
                self._filtered_roll = None

            self._state = self.STATE_ACTIVE
            self._control_enabled.set()

            print(f"\n[Dynamic] Set setpoint: depth={self.target_depth:.2f}m, "
                  f"speed={self.target_speed:.2f}m/s, "
                  f"heading={math.degrees(self.target_heading):.1f}deg")

            return {
                "status": "ok",
                "command": "set_setpoint",
                "setpoints": {
                    "depth": self.target_depth,
                    "speed": self.target_speed,
                    "heading": self.target_heading
                },
                "state": self._state,
                "timestamp": timestamp
            }

    def _cmd_stop_control(self, timestamp: float) -> dict:
        """Stop control and send zero commands"""
        with self._state_lock:
            if self._state == self.STATE_IDLE:
                return {
                    "status": "ok",
                    "command": "stop_control",
                    "message": "controller already idle",
                    "state": self._state,
                    "timestamp": timestamp
                }

            self._state = self.STATE_PAUSED
            self._control_enabled.clear()
            self._filtered_depth = None
            self._filtered_speed = None
            self._filtered_heading = None
            self._filtered_pitch = None
            self._filtered_roll = None

            print("\n[Dynamic] Stopping control, sending zero commands...")
            for _ in range(10):
                self._send_zero_commands()
                time.sleep(0.05)

            return {
                "status": "ok",
                "command": "stop_control",
                "state": self._state,
                "timestamp": timestamp
            }

    def _cmd_get_status(self, timestamp: float) -> dict:
        """Get current controller status"""
        with self._state_lock:
            odom = self._odom_subscriber.latest_data if self._odom_subscriber else None

            status = {
                "status": "ok",
                "command": "get_status",
                "state": self._state,
                "control_mode": self._current_mode,
                "setpoints": {
                    "depth": self.target_depth,
                    "speed": self.target_speed,
                    "heading": self.target_heading
                },
                "timestamp": timestamp
            }

            if odom:
                status["feedback"] = {
                    "depth": odom.depth,
                    "speed": odom.speed,
                    "heading": odom.heading,
                    "pitch": odom.pitch
                }

            return status

    def _cmd_shutdown(self, timestamp: float) -> dict:
        """Shutdown the controller"""
        print("\n[Dynamic] Shutdown requested...")
        self._cmd_stop_control(timestamp)
        self._shutdown_event.set()
        self._running = False

        return {
            "status": "ok",
            "command": "shutdown",
            "message": "shutdown initiated",
            "timestamp": timestamp
        }

    def _reset_pid_controllers(self):
        """Reset all PID controllers"""
        if self._speed_pid:
            self._speed_pid.reset()
        if self._heading_pid:
            self._heading_pid.reset()
        if self._depth_pitch_pid:
            self._depth_pitch_pid.reset()
        if self._pitch_pid:
            self._pitch_pid.reset()
        if self._roll_pid:
            self._roll_pid.reset()

    def _is_dangerous_state(self, odom: OdometryData) -> tuple[bool, str]:
        """Detect dangerous states that may lead to simulation crash"""
        reasons = []

        if abs(odom.vel_y) > self.DANGEROUS_VEL_Y_THRESHOLD:
            reasons.append(f"vel_y={odom.vel_y:.2f}")

        if hasattr(self, '_last_roll_for_rate') and hasattr(self, '_last_roll_time'):
            dt = odom.timestamp - self._last_roll_time
            if dt > 0 and dt < 1.0:
                roll_diff = odom.roll - self._last_roll_for_rate
                while roll_diff > math.pi:
                    roll_diff -= 2 * math.pi
                while roll_diff < -math.pi:
                    roll_diff += 2 * math.pi
                roll_rate = abs(roll_diff) / dt
                if roll_rate > self.DANGEROUS_ROLL_RATE_THRESHOLD:
                    reasons.append(f"roll_rate={roll_rate:.2f}")

        self._last_roll_for_rate = odom.roll
        self._last_roll_time = odom.timestamp

        return len(reasons) > 0, ", ".join(reasons)

    def _compute_roll_stabilization(self, roll: float, current_time: float) -> float:
        """Compute differential RPM for roll stabilization"""
        if not self.ROLL_STABILIZATION_ENABLED:
            return 0.0

        # Apply deadzone - no correction for small roll angles
        if abs(roll) < self.ROLL_DEADZONE:
            # Reset PID integral when within deadzone to prevent accumulation
            if self._roll_pid:
                self._roll_pid._integral = 0.0
            return 0.0

        return self._roll_pid.update(0.0, roll, current_time)

    def _control_iteration(self, current_time: float, elapsed: float, count: int = 0) -> bool:
        """Execute a single control iteration."""
        odom = self._odom_subscriber.latest_data
        if odom is None:
            return False

        # Sensor sanity check
        prev_depth = self._odom_data.depth
        prev_speed = self._odom_data.speed
        dt = current_time - getattr(self, '_last_sensor_time', current_time)
        self._last_sensor_time = current_time
        self._last_control_dt = dt

        sensor_valid = True
        if abs(odom.depth) > self.SENSOR_DEPTH_MAX:
            print(f"\n[{elapsed:.1f}s] WARNING: Abnormal depth {odom.depth:.1f}m")
            sensor_valid = False
        if abs(odom.speed) > self.SENSOR_SPEED_MAX:
            print(f"\n[{elapsed:.1f}s] WARNING: Abnormal speed {odom.speed:.1f}m/s")
            sensor_valid = False

        if dt > 0 and dt < 1.0:
            depth_rate = abs(odom.depth - prev_depth) / dt
            speed_rate = abs(odom.speed - prev_speed) / dt
            if depth_rate > self.SENSOR_RATE_MAX_DEPTH:
                print(f"\n[{elapsed:.1f}s] WARNING: Abnormal depth rate {depth_rate:.1f}m/s")
                sensor_valid = False
            if speed_rate > self.SENSOR_RATE_MAX_SPEED:
                print(f"\n[{elapsed:.1f}s] WARNING: Abnormal speed rate {speed_rate:.1f}m/s^2")
                sensor_valid = False

        if not sensor_valid:
            self._send_zero_commands()
            return False

        self._odom_data = odom

        # EMA low-pass filter: y = α·x + (1-α)·y_prev
        if self._filtered_depth is None:
            # First valid reading — initialize filter state
            self._filtered_depth = odom.depth
            self._filtered_speed = odom.speed
            self._filtered_heading = odom.heading
            self._filtered_pitch = odom.pitch
            self._filtered_roll = odom.roll
        else:
            self._filtered_depth = self.FILTER_DEPTH_ALPHA * odom.depth + (1 - self.FILTER_DEPTH_ALPHA) * self._filtered_depth
            self._filtered_speed = self.FILTER_SPEED_ALPHA * odom.speed + (1 - self.FILTER_SPEED_ALPHA) * self._filtered_speed
            # Heading: use angle-aware EMA to handle wraparound
            h_err = AnglePIDController.normalize_angle(odom.heading - self._filtered_heading)
            self._filtered_heading = AnglePIDController.normalize_angle(
                self._filtered_heading + self.FILTER_HEADING_ALPHA * h_err)
            # Pitch: use angle-aware EMA
            p_err = AnglePIDController.normalize_angle(odom.pitch - self._filtered_pitch)
            self._filtered_pitch = AnglePIDController.normalize_angle(
                self._filtered_pitch + self.FILTER_PITCH_ALPHA * p_err)
            # Roll: use angle-aware EMA
            r_err = AnglePIDController.normalize_angle(odom.roll - self._filtered_roll)
            self._filtered_roll = AnglePIDController.normalize_angle(
                self._filtered_roll + self.FILTER_ROLL_ALPHA * r_err)

        depth = self._filtered_depth
        speed = self._filtered_speed
        heading = self._filtered_heading
        pitch = self._filtered_pitch
        roll = self._filtered_roll

        # Dangerous state detection
        is_dangerous, danger_reason = self._is_dangerous_state(odom)
        if is_dangerous:
            if not getattr(self, '_in_dangerous_state', False):
                print(f"\n[{elapsed:.1f}s] WARNING: Dangerous state! {danger_reason}")
                self._in_dangerous_state = True
        else:
            if getattr(self, '_in_dangerous_state', False):
                print(f"\n[{elapsed:.1f}s] Dangerous state cleared.")
            self._in_dangerous_state = False

        # Roll stabilization
        diff_rpm = self._compute_roll_stabilization(roll, current_time)

        # Mode determination (depth-based only)
        new_mode = self._determine_mode(depth)

        if new_mode != self._current_mode:
            self._on_mode_switch(self._current_mode, new_mode)
            self._current_mode = new_mode

        # Diagnostic data
        diag_data = {
            'phase': 'unknown',
            'pitch_feedforward': 0.0,
            'pitch_correction': 0.0,
            'pitch_limit_active': 0.0,
            'vbs_gain': 0.0,
            'vbs_adjustment': 0.0,
            'lcg_contribution': 0.0,
            'speed_pid_output': 0.0,
            'heading_pid_output': 0.0,
            'depth_pid_output': 0.0,
            'pitch_pid_output': 0.0,
            'reached_target': self._reached_target_depth,
            'roll_target': 0.0,
            'roll_error': self._roll_pid.angle_error(0.0, roll) if self._roll_pid else 0.0,
            'roll_diff_rpm': diff_rpm,
            'rpm1': self._last_rpm1,
            'rpm2': self._last_rpm2,
            'roll_pid_integral': self._roll_pid.integral if self._roll_pid else 0.0,
            'in_dangerous_state': getattr(self, '_in_dangerous_state', False),
            'vel_y': odom.vel_y,
        }

        depth_err = self.target_depth - depth

        if self._current_mode == self.MODE_DIVE:
            # ========== DIVE MODE ==========
            # VBS: negative buoyancy for sinking
            # LCG: rear position for nose-down
            # RPM: low speed for gentle forward motion
            # Pitch: controlled nose-down angle

            diag_data['phase'] = 'dive'

            # VBS for negative buoyancy (sinking)
            self._send_vbs_command(self.DIVE_VBS_SETPOINT)

            # LCG rear for nose-down attitude
            self._send_lcg_command(self.DIVE_LCG_SETPOINT)

            # Speed control - low speed during dive
            rpm = self._speed_pid.update(self.DIVE_SPEED_TARGET, speed, current_time)
            diag_data['speed_pid_output'] = rpm
            self._send_thruster_command(rpm, diff_rpm)

            # Pitch control for dive angle
            self._last_pitch_sp = self.DIVE_PITCH_TARGET
            elevator = self._pitch_pid.update(self.DIVE_PITCH_TARGET, pitch, current_time)
            diag_data['pitch_pid_output'] = elevator
            diag_data['pitch_err'] = self._pitch_pid.angle_error(self.DIVE_PITCH_TARGET, pitch)

            # No heading control during dive (rudder = 0)
            self._send_thrust_angle_command(0.0, elevator)
            diag_data['heading_pid_output'] = 0.0

        else:
            # ========== CRUISE MODE ==========
            # VBS: neutral buoyancy (50%)
            # Depth control via pitch angle + small VBS adjustment
            # Speed control via RPM
            # Heading control via rudder

            # Determine steady-state vs descent phase
            if self._in_steady_phase:
                if abs(depth_err) > self.DEPTH_STEADY_THRESHOLD + self.DEPTH_STEADY_HYSTERESIS:
                    self._in_steady_phase = False
                    print(f"\n[{elapsed:.1f}s] Phase: steady -> descent (err={depth_err:.2f}m)")
            else:
                if abs(depth_err) < self.DEPTH_STEADY_THRESHOLD:
                    self._in_steady_phase = True
                    print(f"\n[{elapsed:.1f}s] Phase: descent -> steady (err={depth_err:.2f}m)")

            if self._in_steady_phase:
                diag_data['phase'] = 'cruise_steady'
                current_pitch_limit = self.PITCH_LIMIT_STEADY
                current_feedforward = self.PITCH_FEEDFORWARD_PER_SPEED_STEADY
                vbs_gain = 35.0
                vbs_limit = 40.0
            else:
                diag_data['phase'] = 'cruise_descent'
                current_pitch_limit = self.PITCH_LIMIT_DESCENT
                current_feedforward = self.PITCH_FEEDFORWARD_PER_SPEED_DESCENT
                vbs_gain = 30.0
                vbs_limit = 45.0

            diag_data['pitch_limit_active'] = current_pitch_limit
            diag_data['vbs_gain'] = vbs_gain
            diag_data['in_steady_phase'] = self._in_steady_phase

            # Smooth heading setpoint to prevent aggressive turns
            heading_error_to_target = self._heading_pid.angle_error(self.target_heading, self._smoothed_heading)
            if abs(heading_error_to_target) > self.HEADING_RATE_LIMIT:
                # Limit heading change rate
                heading_step = self.HEADING_RATE_LIMIT if heading_error_to_target > 0 else -self.HEADING_RATE_LIMIT
                self._smoothed_heading = self._heading_pid.normalize_angle(self._smoothed_heading + heading_step)
            else:
                self._smoothed_heading = self.target_heading

            # Heading control (only in cruise mode, and only when not in early descent)
            if depth_err > 2.0:
                rudder = 0.0  # No heading control during early descent
            else:
                rudder = self._heading_pid.update(self._smoothed_heading, heading, current_time)
                if getattr(self, '_in_dangerous_state', False):
                    rudder = rudder * self.DANGEROUS_STATE_CONTROL_FACTOR
            diag_data['heading_pid_output'] = rudder
            diag_data['smoothed_heading'] = self._smoothed_heading
            diag_data['smoothed_heading_deg'] = math.degrees(self._smoothed_heading)

            # Pitch feedforward
            pitch_feedforward = current_feedforward * speed
            diag_data['pitch_feedforward'] = pitch_feedforward

            # Pitch correction from depth PID
            pitch_correction = -self._depth_pitch_pid.update(self.target_depth, depth, current_time)
            diag_data['pitch_correction'] = pitch_correction
            diag_data['depth_pid_output'] = -pitch_correction

            # Combined pitch setpoint
            target_pitch = pitch_feedforward + pitch_correction
            target_pitch = max(-current_pitch_limit, min(current_pitch_limit, target_pitch))
            self._last_pitch_sp = target_pitch

            # Inner pitch loop
            elevator = self._pitch_pid.update(target_pitch, pitch, current_time)
            diag_data['pitch_pid_output'] = elevator
            diag_data['pitch_err'] = self._pitch_pid.angle_error(target_pitch, pitch)
            self._send_thrust_angle_command(rudder, elevator)

            # LCG assists pitch control
            lcg_contribution = (elevator / self.ELEVATOR_MAX) * 50.0 * self.LCG_PITCH_RATIO
            diag_data['lcg_contribution'] = lcg_contribution
            self._send_lcg_command(self.LCG_CENTER + lcg_contribution)

            # VBS: neutral buoyancy with small depth correction
            if self._in_steady_phase:
                vbs_integral_gain = 10.0
                self._vbs_integral += depth_err * self.control_interval * vbs_integral_gain
                self._vbs_integral = max(-15.0, min(15.0, self._vbs_integral))
            else:
                self._vbs_integral *= 0.9

            vbs_depth_adjustment = depth_err * vbs_gain + self._vbs_integral
            diag_data['vbs_adjustment'] = vbs_depth_adjustment
            vbs_output = self.CRUISE_VBS_NEUTRAL + max(-vbs_limit, min(vbs_limit, vbs_depth_adjustment))
            self._send_vbs_command(vbs_output)

            # Speed control
            rpm = self._speed_pid.update(self.target_speed, speed, current_time)
            diag_data['speed_pid_output'] = rpm
            self._send_thruster_command(rpm, diff_rpm)

        # Calculate errors for logging
        depth_err = self.target_depth - depth
        speed_err = self.target_speed - speed
        heading_err = self._heading_pid.angle_error(self.target_heading, heading)

        self._record_data(elapsed, depth_err, speed_err, heading_err, diag_data,
                          dt=getattr(self, '_last_control_dt', 0.0), count=count)

        return True

    def run(self):
        """Main entry point - always runs in dynamic mode"""
        print("=== DEPTH CRUISE CONTROLLER ===")
        print(f"Service address: {self._service_addr}")
        print("Waiting for setpoint commands...")
        print()

        # Setup
        self._setup_pid_controllers()
        self._setup_subscribers()
        self._setup_publishers()

        # Start sensor subscriber
        self._odom_subscriber.start()

        # Connect publishers
        self._thruster1_pub.connect()
        self._thruster2_pub.connect()
        self._angle_pub.connect()
        self._vbs_pub.connect()
        self._lcg_pub.connect()

        # Wait for initial data
        print("Waiting for sensor data...")
        time.sleep(1.0)

        if not self._odom_subscriber.is_connected:
            print("WARNING: No sensor data received! Make sure simulation is running.")
        else:
            print("Sensor data received.")

        # Start service
        self._service = Replier()
        self._service.init(self._service_addr)
        self._service.set_handler(self._handle_request)
        self._service.start()
        print(f"Service started on {self._service_addr}")

        # Start control thread
        self._running = True
        self._start_time = time.time()
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

        print()
        print("Available commands:")
        print("  set_setpoint: Set depth/speed/heading and start control")
        print("  stop_control: Stop control and send zero commands")
        print("  get_status:   Get current controller status")
        print("  shutdown:     Shutdown the controller")
        print()

        # Wait for shutdown signal
        try:
            while not self._shutdown_event.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received...")

        # Cleanup
        self._cleanup()

    def _control_loop(self):
        """Control loop"""
        last_control_time = time.time()
        last_print_time = 0
        loop_count = 0

        while self._running and not self._shutdown_event.is_set():
            if not self._control_enabled.wait(timeout=0.1):
                continue

            current_time = time.time()

            with self._state_lock:
                if self._state != self.STATE_ACTIVE:
                    continue

            if current_time - last_control_time >= self.control_interval:
                last_control_time = current_time
                elapsed = current_time - self._start_time
                loop_count += 1

                if self._control_iteration(current_time, elapsed, count=loop_count):
                    if current_time - last_print_time >= 1.0:
                        last_print_time = current_time
                        odom = self._odom_data
                        depth_err = self.target_depth - odom.depth
                        speed_err = self.target_speed - odom.speed
                        heading_err = self._heading_pid.angle_error(self.target_heading, odom.heading)
                        print(f"[{elapsed:.1f}s] {self._current_mode:<6} "
                              f"D:{self.target_depth:.1f}/{odom.depth:.1f}/{depth_err:+.2f}m  "
                              f"S:{self.target_speed:.1f}/{odom.speed:.2f}/{speed_err:+.2f}m/s  "
                              f"H:{math.degrees(self.target_heading):.0f}/{math.degrees(odom.heading):.0f}deg  "
                              f"RPM:{self._last_rpm:.0f}")

            time.sleep(0.001)

    def _cleanup(self):
        """Cleanup resources"""
        print("\nCleaning up...")
        self._running = False
        self._control_enabled.set()

        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)

        if self._service:
            self._service.stop()

        print("Sending zero commands...")
        for _ in range(10):
            self._send_zero_commands()
            time.sleep(0.05)

        if self._odom_subscriber:
            self._odom_subscriber.stop()

        if self._thruster1_pub:
            self._thruster1_pub.close()
        if self._thruster2_pub:
            self._thruster2_pub.close()
        if self._angle_pub:
            self._angle_pub.close()
        if self._vbs_pub:
            self._vbs_pub.close()
        if self._lcg_pub:
            self._lcg_pub.close()

        filepath = self._save_data()
        if filepath:
            print(f"Data saved to: {filepath}")

        print("Controller stopped.")


def main():
    default_config = os.path.join(_SCRIPT_DIR, '..', 'config', 'depth_cruise_controller.jsonc')

    parser = argparse.ArgumentParser(
        description='SAM-AUV Depth Cruise Controller (Two-Mode Design)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Start controller (waits for setpoint via service)
    python depth_cruise_controller.py

    # Custom config file
    python depth_cruise_controller.py --config path/to/depth_cruise_controller.jsonc

Control commands (via NNG service):
    set_setpoint: {"command": "set_setpoint", "depth": 5.0, "speed": 1.5, "heading": 0.0}
    stop_control: {"command": "stop_control"}
    get_status:   {"command": "get_status"}
    shutdown:     {"command": "shutdown"}

Control Modes:
    - DIVE mode:   Initial descent from surface (depth < 2.0m)
                   VBS=75%%, LCG=70%%, low RPM, nose-down pitch
    - CRUISE mode: Stable cruise at target depth (depth >= 2.0m)
                   VBS=50%% (neutral), pitch-based depth control
        """
    )

    parser.add_argument('--config', type=str, default=default_config,
                        help='Path to JSONC config file (default: ../config/depth_cruise_controller.jsonc)')

    args = parser.parse_args()

    controller = DepthCruiseController(config_path=args.config)
    controller.run()


if __name__ == '__main__':
    main()
