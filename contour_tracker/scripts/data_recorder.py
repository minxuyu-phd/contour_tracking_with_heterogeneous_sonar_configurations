#!/usr/bin/env python3
"""
Odometry Data Recorder

Subscribes to AUV odometry data and records it to CSV at 1-second intervals.

Usage:
    python odom_recorder.py
    python odom_recorder.py --addr tcp://192.168.5.11:5555
"""

import argparse
import csv
import json
import os
import signal
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from nng_comm.scripts import Subscriber

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Default addresses
DEFAULT_ODOM_ADDR = "tcp://192.168.5.11:5555"
DEFAULT_TRAJ_ADDR = "tcp://192.168.5.11:6600"

# Global running flag
_running = True


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


def _signal_handler(sig, frame):
    global _running
    print("\nReceived shutdown signal, stopping...")
    _running = False


class OdomRecorder:
    """Subscribe to odometry data and record to CSV at 1-second intervals."""

    def __init__(self, addr: str, traj_addr: str):
        self._addr = addr
        self._traj_addr = traj_addr
        self._sub = Subscriber()
        self._traj_sub = Subscriber()
        self._lock = threading.Lock()
        self._latest_data = None
        self._recv_count = 0
        self._collection_done = False

    def _on_message(self, msg: dict):
        with self._lock:
            self._recv_count += 1
            self._latest_data = msg

    def _on_trajectory(self, msg: dict):
        points = msg.get("points")
        if points is not None and len(points) == 0:
            self._collection_done = True

    @property
    def latest_data(self):
        with self._lock:
            return self._latest_data

    @property
    def is_connected(self) -> bool:
        return self._recv_count > 0

    @property
    def collection_done(self) -> bool:
        return self._collection_done

    def start(self):
        if not self._sub.init(self._addr):
            print(f"Failed to connect to {self._addr}")
            return False
        self._sub.set_callback(self._on_message)
        self._sub.start_async()

        if not self._traj_sub.init(self._traj_addr):
            print(f"Failed to connect to trajectory {self._traj_addr}")
            self._sub.close()
            return False
        self._traj_sub.set_callback(self._on_trajectory)
        self._traj_sub.start_async()
        return True

    def stop(self):
        self._traj_sub.close()
        self._sub.close()


def extract_row(msg: dict) -> list:
    """Extract a CSV row from an odometry message."""
    pos = msg.get('position', {})
    vel = msg.get('linear_velocity', {})
    ori = msg.get('orientation', {})
    return [
        msg.get('timestamp', time.time()),
        pos.get('x', 0.0),
        pos.get('y', 0.0),
        pos.get('z', 0.0),
        vel.get('x', 0.0),
        vel.get('y', 0.0),
        vel.get('z', 0.0),
        ori.get('roll', 0.0),
        ori.get('pitch', 0.0),
        ori.get('yaw', 0.0),
    ]


CSV_HEADER = ['timestamp', 'pos_x', 'pos_y', 'pos_z',
              'vel_x', 'vel_y', 'vel_z',
              'roll', 'pitch', 'yaw']


def main():
    global _running

    parser = argparse.ArgumentParser(description='Record AUV odometry data to CSV')
    parser.add_argument('--config', type=str,
                        default=os.path.join(_SCRIPT_DIR, '..', 'config', 'data_recorder.jsonc'),
                        help='Path to JSONC config file')
    parser.add_argument('--addr', type=str, default=None,
                        help=f'Odometry address override (default from config)')
    args = parser.parse_args()

    # Load config
    cfg = {}
    if os.path.exists(args.config):
        cfg = load_jsonc(args.config)
        print(f"Loaded config: {args.config}")
    else:
        print(f"Config not found: {args.config}, using defaults")

    odom_addr = args.addr or cfg.get("odometry_address", DEFAULT_ODOM_ADDR)
    traj_addr = cfg.get("trajectory_address", DEFAULT_TRAJ_ADDR)

    # Signal handlers
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Prepare output directory and file
    data_dir = os.path.join(_SCRIPT_DIR, 'data')
    os.makedirs(data_dir, exist_ok=True)

    filename = datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
    filepath = os.path.join(data_dir, filename)

    # Start subscriber
    recorder = OdomRecorder(odom_addr, traj_addr)
    if not recorder.start():
        sys.exit(1)

    print(f"=== Odometry Recorder ===")
    print(f"Odometry:       {odom_addr}")
    print(f"Trajectory:     {traj_addr}")
    print(f"Output file:    {filepath}")
    print(f"Waiting for first odometry message...")

    # Wait for first message
    while _running and not recorder.is_connected:
        time.sleep(0.1)

    if not _running:
        recorder.stop()
        print("Stopped before receiving any data.")
        return

    print("First message received, recording started.")

    # Open CSV and start recording
    row_count = 0
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)

        while _running:
            if recorder.collection_done:
                print("\n轨迹闭合完成，数据采集结束")
                break
            data = recorder.latest_data
            if data is not None:
                writer.writerow(extract_row(data))
                row_count += 1
                if row_count % 10 == 0:
                    f.flush()
            time.sleep(1.0)

    # Cleanup
    recorder.stop()
    print(f"Recording finished.")
    print(f"  Rows recorded: {row_count}")
    print(f"  File: {filepath}")


if __name__ == '__main__':
    main()
