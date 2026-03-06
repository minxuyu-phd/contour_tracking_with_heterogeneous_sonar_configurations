#!/usr/bin/env python3
"""
Test script to subscribe to SAM-AUV sensor data
Uses nng_comm Subscriber wrapper.
"""

import sys
import os
import json
import signal
import math
from datetime import datetime

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, '..', '..', 'nng_comm'))
from scripts import Subscriber

running = True

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

def rad_to_deg(rad: float) -> float:
    """Convert radians to degrees in range (-180, 180]"""
    deg = math.degrees(rad)
    # Normalize to (-180, 180]
    while deg > 180:
        deg -= 360
    while deg <= -180:
        deg += 360
    return deg

def format_angle_fields(msg: dict, sensor_name: str) -> dict:
    """Format angle fields from radians to degrees for display

    Returns a new dict with angle fields converted to degrees string format.
    Handles nested structures like orientation: {roll, pitch, yaw}
    """
    import copy
    result = copy.deepcopy(msg)

    # Define which fields are angles (in radians) for each sensor type
    # Format: field_name or nested.field_name
    angle_fields = {
        'odometry': ['orientation.roll', 'orientation.pitch', 'orientation.yaw'],
        'imu': ['roll', 'pitch', 'yaw',
                'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z'],
        'dvl': [],  # DVL typically doesn't have angle fields
        'pressure': [],
        'gps': [],
        'msis': [],  # MSIS angle is already in degrees
        'echosounder_left': [],
        'echosounder_right': [],
    }

    # Angular velocity fields (rad/s -> deg/s)
    angular_velocity_patterns = ['angular_velocity']

    fields_to_convert = angle_fields.get(sensor_name, [])

    for field_path in fields_to_convert:
        parts = field_path.split('.')

        # Navigate to the parent container
        container = result
        for part in parts[:-1]:
            if isinstance(container, dict) and part in container:
                container = container[part]
            else:
                container = None
                break

        if container is None:
            continue

        field = parts[-1]
        if field in container and isinstance(container[field], (int, float)):
            rad_val = container[field]

            # Check if this is an angular velocity field
            is_angular_velocity = any(pattern in field_path for pattern in angular_velocity_patterns)

            if is_angular_velocity:
                # Angular velocity: just convert rad/s to deg/s
                deg_val = math.degrees(rad_val)
                container[field] = f"{deg_val:.2f} deg/s ({rad_val:.4f} rad/s)"
            else:
                # Angle: normalize to (-180, 180]
                deg_val = rad_to_deg(rad_val)
                container[field] = f"{deg_val:.2f} deg ({rad_val:.4f} rad)"

    return result

def signal_handler(sig, frame):
    global running
    print("\nReceived Ctrl+C, shutting down...")
    running = False

def subscribe_sensor(name, address):
    """Subscribe to a sensor and print received messages"""
    global running
    signal.signal(signal.SIGINT, signal_handler)

    print(f"Subscribing to {name} at {address}")

    try:
        with Subscriber() as sub:
            if not sub.init(address):
                print(f"Error: failed to connect to {address}")
                sys.exit(1)

            print(f"Connected to {name}, waiting for messages...")
            print("Press Ctrl+C to stop\n")

            count = 0
            while running:
                msg = sub.receive(timeout_ms=1000)
                if msg is None:
                    continue

                count += 1
                ts = datetime.fromtimestamp(msg.get('timestamp', 0))

                # Format angle fields for display
                display_msg = format_angle_fields(msg, name)

                # For MSIS, truncate beam_data for display
                if name == 'msis' and 'beam_data' in display_msg:
                    beam_data = display_msg.get('beam_data')
                    if beam_data is not None and isinstance(beam_data, (list, bytes)):
                        data_len = len(beam_data) if isinstance(beam_data, (list, bytes)) else 0
                        display_msg['beam_data'] = f"<{data_len} bytes>"

                print(f"[{count}] {name} @ {ts.strftime('%H:%M:%S.%f')[:-3]}")
                print(json.dumps(display_msg, indent=2, default=str))
                print("-" * 60)

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

    print(f"Stopped subscribing to {name}")

def main():
    config_path = os.path.join(_SCRIPT_DIR, '..', 'config', 'test_subscribe.jsonc')
    sensors = load_jsonc(config_path)["sensors"]

    if len(sys.argv) > 1:
        sensor_name = sys.argv[1]
        if sensor_name not in sensors:
            print(f"Unknown sensor: {sensor_name}")
            print(f"Available sensors: {', '.join(sensors.keys())}")
            sys.exit(1)

        subscribe_sensor(sensor_name, sensors[sensor_name])
    else:
        print("Usage: test_subscribe.py <sensor_name>")
        print(f"Available sensors: {', '.join(sensors.keys())}")
        print("\nExample: test_subscribe.py odometry")

if __name__ == '__main__':
    main()
