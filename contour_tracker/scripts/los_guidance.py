#!/usr/bin/env python3
"""
LOS Guidance Controller for SAM-AUV

Subscribes to reference trajectory from contour_fitting.py and odometry from
simulation, computes desired heading using Line-of-Sight (LOS) guidance, then
sends setpoints to the low-level PID controller via NNG Req/Rep.

Architecture:
    contour_fitting.py --Pub--> trajectory_pts --Sub--> LOS Guidance
    Simulation         --Pub--> odometry       --Sub--> LOS Guidance
    LOS Guidance       --Req--> set_setpoint   --Rep--> depth_cruise_controller

Usage:
    python los_guidance.py --config los_guidance.json
    python los_guidance.py --lookahead 15 --speed 1.2 --depth 8
"""

import argparse
import atexit
import json
import math
import os
import signal
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from nng_comm.scripts import Subscriber, Requester


class LOSGuidance:
    """
    LOS (Line-of-Sight) path tracking controller.

    Finds the closest point on the reference trajectory, then looks ahead
    by a configurable distance along the path to compute the desired heading.
    Sends (speed, heading, depth) setpoints to the PID controller.
    """

    def __init__(self, cfg: dict):
        # Configuration
        self._odom_addr = cfg.get("odometry_address", "tcp://192.168.5.11:5555")
        self._traj_addr = cfg.get("trajectory_address", "tcp://192.168.5.11:6600")
        self._pid_addr = cfg.get("pid_service_address", "tcp://192.168.5.11:7780")
        self._rate_hz = cfg.get("rate_hz", 10)
        self._control_interval = 1.0 / self._rate_hz
        self.lookahead = cfg.get("lookahead_distance", 10.0)
        self.cruise_speed = cfg.get("cruise_speed", 1.5)
        self.cruise_depth = cfg.get("cruise_depth", 5.0)

        # AUV state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading = 0.0
        self.speed = 0.0

        # Reference trajectory (Nx2 numpy array)
        self._trajectory = None

        # NNG sockets
        self._odom_sub = None
        self._traj_sub = None
        self._pid_req = None

        # Running flag
        self._running = False
        self._sigint_count = 0
        self._closure_stopped = False

        # Statistics
        self._loop_count = 0
        self._last_print_time = 0.0
        self._last_los_pt = None
        self._last_cte = 0.0

        # Register cleanup
        atexit.register(self._cleanup_atexit)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _cleanup_atexit(self):
        """Cleanup handler registered with atexit"""
        self._disconnect()

    def _signal_handler(self, sig, frame):
        """Handle shutdown signals"""
        self._sigint_count += 1
        self._running = False

        if self._sigint_count == 1:
            print("\nReceived shutdown signal, stopping...")
            return

        print("\nForce exit...")
        self._disconnect()
        os._exit(1)

    # ── NNG Connection Management ──

    def _connect(self):
        """Establish NNG connections"""
        self._odom_sub = Subscriber()
        self._odom_sub.init(self._odom_addr)

        self._traj_sub = Subscriber()
        self._traj_sub.init(self._traj_addr)

        self._pid_req = Requester()
        self._pid_req.init(self._pid_addr)

        print(f"Connected to odometry:   {self._odom_addr}")
        print(f"Connected to trajectory: {self._traj_addr}")
        print(f"Connected to PID:        {self._pid_addr}")

    def _disconnect(self):
        """Close NNG connections"""
        if self._odom_sub is not None:
            self._odom_sub.close()
            self._odom_sub = None
        if self._traj_sub is not None:
            self._traj_sub.close()
            self._traj_sub = None
        if self._pid_req is not None:
            self._pid_req.close()
            self._pid_req = None

    # ── State Update ──

    def _update_state(self) -> bool:
        """
        Update AUV state from odometry.

        Returns True if state updated successfully.
        """
        if self._sigint_count > 0:
            return False

        odom_sub = self._odom_sub
        if odom_sub is None:
            return False

        msg = odom_sub.receive(timeout_ms=20)
        if msg is None:
            return False

        if 'position' in msg:
            self.x = msg['position'].get('x', 0.0)
            self.y = msg['position'].get('y', 0.0)
            self.z = msg['position'].get('z', 0.0)

        if 'linear_velocity' in msg:
            self.speed = msg['linear_velocity'].get('x', 0.0)

        if 'orientation' in msg:
            self.heading = msg['orientation'].get('yaw', 0.0)

        return True

    def _update_trajectory(self) -> bool:
        """
        Update reference trajectory from contour_fitting publisher.

        Drains the subscriber queue and keeps only the latest message.
        Returns True if a new trajectory was received.
        """
        traj_sub = self._traj_sub
        if traj_sub is None:
            return False

        latest = None
        while True:
            msg = traj_sub.receive(timeout_ms=20)
            if msg is None:
                break
            latest = msg

        if latest is None:
            return False

        pts = latest.get('points')
        if pts is not None and len(pts) >= 2:
            self._trajectory = np.array(pts, dtype=np.float64)
            return True

        # 收到空轨迹，清除轨迹并标记停止
        if pts is not None and len(pts) == 0:
            self._trajectory = None
            self._closure_stopped = True
            return True

        return False

    # ── LOS Algorithm ──

    def _compute_los_heading(self):
        """
        Compute desired heading using LOS guidance.

        1. Find the closest point on the trajectory to the AUV.
        2. Walk forward along the path by lookahead distance.
        3. Return atan2 heading towards the lookahead point.

        Returns heading in radians, or None if no trajectory available.
        """
        traj = self._trajectory
        if traj is None or len(traj) < 2:
            return None

        # 1. Closest point on trajectory
        dists = np.linalg.norm(traj - np.array([self.x, self.y]), axis=1)
        idx = int(np.argmin(dists))
        self._last_cte = float(dists[idx])

        # 2. Walk along path from closest point to find lookahead point
        accumulated = 0.0
        los_pt = traj[-1]  # fallback: path endpoint

        for i in range(idx, len(traj) - 1):
            seg = float(np.linalg.norm(traj[i + 1] - traj[i]))
            accumulated += seg
            if accumulated >= self.lookahead:
                # Interpolate to exact lookahead distance on this segment
                overshoot = accumulated - self.lookahead
                ratio = 1.0 - overshoot / seg if seg > 1e-9 else 1.0
                los_pt = traj[i] + ratio * (traj[i + 1] - traj[i])
                break

        self._last_los_pt = los_pt

        # 3. Desired heading
        return math.atan2(los_pt[1] - self.y, los_pt[0] - self.x)

    # ── PID Communication ──

    def _send_setpoint(self, speed: float, heading: float, depth: float) -> bool:
        """
        Send setpoint to PID controller.

        Args:
            speed: Target speed (m/s)
            heading: Target heading (radians)
            depth: Target depth (m)

        Returns True if sent successfully.
        """
        if not self._running:
            return False

        pid_req = self._pid_req
        if pid_req is None:
            return False

        request = {
            "command": "set_setpoint",
            "depth": depth,
            "speed": speed,
            "heading": math.degrees(heading),
            "heading_degrees": True
        }

        response = pid_req.request(request, timeout_ms=1000)
        if response is None:
            if self._running:
                print("Warning: PID service timeout")
            return False
        return response.get('status') == 'ok'

    def _stop_control(self) -> bool:
        """Send stop command to PID controller"""
        pid_req = self._pid_req
        if pid_req is None:
            return False

        response = pid_req.request({"command": "stop_control"}, timeout_ms=1000)
        if response is None:
            return False
        return response.get('status') == 'ok'

    # ── Main Loop ──

    def run(self):
        """Main control loop"""
        print("=== LOS GUIDANCE CONTROLLER ===")
        print(f"Control rate:  {self._rate_hz} Hz")
        print(f"Lookahead:     {self.lookahead} m")
        print(f"Cruise speed:  {self.cruise_speed} m/s")
        print(f"Cruise depth:  {self.cruise_depth} m")
        print()

        # Connect to NNG sockets
        self._connect()

        # Wait for initial odometry
        print("Waiting for odometry data...")
        start_wait = time.time()
        while time.time() - start_wait < 5.0:
            if self._update_state():
                print(f"Initial position: ({self.x:.1f}, {self.y:.1f}, {self.z:.1f}m)")
                break
            time.sleep(0.1)
        else:
            print("Warning: No odometry data received! Proceeding anyway...")

        print("Waiting for trajectory data...")
        print()

        # Main loop
        self._running = True
        last_control_time = time.time()
        start_time = time.time()

        try:
            while self._running:
                if self._sigint_count > 0:
                    break

                current_time = time.time()

                if current_time - last_control_time >= self._control_interval:
                    last_control_time = current_time
                    self._loop_count += 1

                    # Update state
                    self._update_state()

                    # Update trajectory
                    self._update_trajectory()

                    # 收到空轨迹（闭合完成），停止控制
                    if self._closure_stopped:
                        print("[LOS] 收到空轨迹，轮廓闭合完成，停止控制")
                        self._stop_control()
                        self._running = False
                        break

                    # Compute and send setpoint if we have a trajectory
                    if self._trajectory is not None:
                        heading_d = self._compute_los_heading()
                        if heading_d is not None:
                            self._send_setpoint(
                                self.cruise_speed, heading_d, self.cruise_depth
                            )

                            if self._sigint_count > 0:
                                break

                    # Print status at 1 Hz
                    if current_time - self._last_print_time >= 1.0:
                        self._last_print_time = current_time
                        elapsed = current_time - start_time

                        if self._trajectory is not None:
                            los_info = ""
                            if self._last_los_pt is not None:
                                los_info = (
                                    f"  LOS:({self._last_los_pt[0]:.1f},"
                                    f"{self._last_los_pt[1]:.1f})"
                                )
                            print(
                                f"[{elapsed:.1f}s] "
                                f"Pos:({self.x:.1f},{self.y:.1f},{self.z:.1f})  "
                                f"Hdg:{math.degrees(self.heading):.0f}deg  "
                                f"Spd:{self.speed:.2f}m/s  "
                                f"CTE:{self._last_cte:.2f}m"
                                f"{los_info}"
                            )
                        else:
                            print(
                                f"[{elapsed:.1f}s] "
                                f"Pos:({self.x:.1f},{self.y:.1f},{self.z:.1f})  "
                                f"Waiting for trajectory..."
                            )

                # Prevent busy waiting
                time.sleep(0.001)

        except KeyboardInterrupt:
            pass

        except Exception as e:
            print(f"\nError in control loop: {e}")
            import traceback
            traceback.print_exc()

        finally:
            print("\nStopping controller...")
            self._stop_control()
            self._disconnect()
            print("Controller stopped.")


def main():
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    default_config = os.path.join(_SCRIPT_DIR, '..', 'config', 'los_guidance.jsonc')

    parser = argparse.ArgumentParser(
        description='LOS Guidance Controller for SAM-AUV',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python los_guidance.py --config los_guidance.jsonc
    python los_guidance.py --lookahead 15 --speed 1.2 --depth 8

Prerequisites:
    1. Start the simulation: cd build && ./sam_auv_sim
    2. Start the PID controller: python depth_cruise_controller.py
    3. Start contour fitting: python contour_fitting.py
    4. Then run this controller
        """
    )

    parser.add_argument('--config', type=str, default=default_config,
                        help='Path to JSON config file (default: los_guidance.json)')
    parser.add_argument('--odom-addr', type=str,
                        help='Odometry address (overrides config)')
    parser.add_argument('--traj-addr', type=str,
                        help='Trajectory address (overrides config)')
    parser.add_argument('--pid-addr', type=str,
                        help='PID service address (overrides config)')
    parser.add_argument('--lookahead', type=float,
                        help='Lookahead distance in meters (overrides config)')
    parser.add_argument('--speed', type=float,
                        help='Cruise speed in m/s (overrides config)')
    parser.add_argument('--depth', type=float,
                        help='Cruise depth in meters (overrides config)')

    args = parser.parse_args()

    # Load config
    cfg = {}
    if args.config:
        config_path = args.config
        if not os.path.isabs(config_path):
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config_path)
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                cfg = json.load(f)
            print(f"Loaded config: {config_path}")
        else:
            print(f"Warning: Config file not found: {config_path}, using defaults")

    # Command-line overrides
    if args.odom_addr:
        cfg['odometry_address'] = args.odom_addr
    if args.traj_addr:
        cfg['trajectory_address'] = args.traj_addr
    if args.pid_addr:
        cfg['pid_service_address'] = args.pid_addr
    if args.lookahead is not None:
        cfg['lookahead_distance'] = args.lookahead
    if args.speed is not None:
        cfg['cruise_speed'] = args.speed
    if args.depth is not None:
        cfg['cruise_depth'] = args.depth

    controller = LOSGuidance(cfg)
    controller.run()
    return 0


if __name__ == '__main__':
    exit(main())
