#!/usr/bin/env python3
"""
Test script to send control commands to SAM-AUV
Uses nng_comm Publisher and Requester wrappers.
"""

import sys
import os
import json
import time
import signal

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, '..', '..', 'nng_comm'))
from scripts import Publisher, Requester


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


running = True

def signal_handler(sig, frame):
    global running
    print("\nReceived Ctrl+C, stopping...")
    running = False

def send_thruster_continuous(address, rpm, rate_hz=10):
    """Send thruster command continuously"""
    interval = 1.0 / rate_hz

    signal.signal(signal.SIGINT, signal_handler)

    print(f"Sending thruster command: {rpm} RPM to {address} at {rate_hz} Hz")
    print("Press Ctrl+C to stop\n")

    try:
        with Publisher() as pub:
            if not pub.init(address):
                print(f"Error: failed to listen on {address}")
                sys.exit(1)
            time.sleep(0.1)  # Wait for connection to establish

            count = 0
            while running:
                cmd = {
                    "timestamp": time.time(),
                    "rpm": float(rpm)
                }

                pub.publish(cmd)
                count += 1
                print(f"\r[{count}] Sent thruster: {rpm} RPM", end='', flush=True)
                time.sleep(interval)

    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)

    print(f"\nStopped. Sent {count} commands.")

def send_vbs_continuous(address, percentage, rate_hz=10):
    """Send VBS command continuously"""
    interval = 1.0 / rate_hz

    signal.signal(signal.SIGINT, signal_handler)

    print(f"Sending VBS command: {percentage}% to {address} at {rate_hz} Hz")
    print("Press Ctrl+C to stop\n")

    try:
        with Publisher() as pub:
            if not pub.init(address):
                print(f"Error: failed to listen on {address}")
                sys.exit(1)
            time.sleep(0.1)

            count = 0
            while running:
                cmd = {
                    "timestamp": time.time(),
                    "percentage": float(percentage)
                }

                pub.publish(cmd)
                count += 1
                print(f"\r[{count}] Sent VBS: {percentage}%", end='', flush=True)
                time.sleep(interval)

    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)

    print(f"\nStopped. Sent {count} commands.")

def send_thrust_angle_continuous(address, yaw, pitch, rate_hz=10):
    """Send thrust angle command continuously"""
    interval = 1.0 / rate_hz

    signal.signal(signal.SIGINT, signal_handler)

    print(f"Sending thrust angle: yaw={yaw}, pitch={pitch} to {address} at {rate_hz} Hz")
    print("Press Ctrl+C to stop\n")

    try:
        with Publisher() as pub:
            if not pub.init(address):
                print(f"Error: failed to listen on {address}")
                sys.exit(1)
            time.sleep(0.1)

            count = 0
            while running:
                cmd = {
                    "timestamp": time.time(),
                    "horizontal_radians": float(yaw),
                    "vertical_radians": float(pitch)
                }

                pub.publish(cmd)
                count += 1
                print(f"\r[{count}] Sent angle: yaw={yaw}, pitch={pitch}", end='', flush=True)
                time.sleep(interval)

    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)

    print(f"\nStopped. Sent {count} commands.")

def send_config_command(address, command, value=None):
    """Send configuration command via Req/Rep (single shot)"""
    print(f"Sending config command: {command} to {address}")

    try:
        with Requester() as req:
            if not req.init(address):
                print(f"Error: failed to connect to {address}")
                sys.exit(1)

            cmd = {"command": command}
            if value is not None:
                cmd["value"] = value

            print(f"Sent: {cmd}")
            reply = req.request(cmd, timeout_ms=5000)
            if reply is None:
                print("Error: no reply (timeout)")
                sys.exit(1)
            print(f"Reply: {reply}")

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


def send_msis_config(address, rotation_min=None, rotation_max=None,
                     range_min=None, range_max=None,
                     gain=None, unidirectional=None, step_multiplier=None):
    """Configure MSIS sensor parameters"""
    cmd = {"command": "configure_msis"}
    if rotation_min is not None and rotation_max is not None:
        cmd["rotation_min"] = float(rotation_min)
        cmd["rotation_max"] = float(rotation_max)
    if range_min is not None:
        cmd["range_min"] = float(range_min)
    if range_max is not None:
        cmd["range_max"] = float(range_max)
    if gain is not None:
        cmd["gain"] = float(gain)
    if unidirectional is not None:
        cmd["unidirectional"] = bool(unidirectional)
    if step_multiplier is not None:
        cmd["step_multiplier"] = int(step_multiplier)

    print(f"Configuring MSIS: {cmd}")

    try:
        with Requester() as req:
            if not req.init(address):
                print(f"Error: failed to connect to {address}")
                sys.exit(1)

            reply = req.request(cmd, timeout_ms=5000)
            if reply is None:
                print("Error: no reply (timeout)")
                sys.exit(1)
            print(f"Reply: {json.dumps(reply, indent=2)}")

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


def get_msis_config(address):
    """Get current MSIS sensor configuration"""
    cmd = {"command": "get_msis_config"}
    print(f"Getting MSIS configuration...")

    try:
        with Requester() as req:
            if not req.init(address):
                print(f"Error: failed to connect to {address}")
                sys.exit(1)

            reply = req.request(cmd, timeout_ms=5000)
            if reply is None:
                print("Error: no reply (timeout)")
                sys.exit(1)
            print(f"MSIS Configuration:")
            print(json.dumps(reply, indent=2))

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

def parse_msis_args(args):
    """Parse MSIS command line arguments"""
    import argparse
    parser = argparse.ArgumentParser(description='Configure MSIS sensor')
    parser.add_argument('--get', action='store_true', help='Get current configuration')
    parser.add_argument('--rotation-min', type=float, help='Minimum rotation angle (degrees)')
    parser.add_argument('--rotation-max', type=float, help='Maximum rotation angle (degrees)')
    parser.add_argument('--range-min', type=float, help='Minimum detection range (meters)')
    parser.add_argument('--range-max', type=float, help='Maximum detection range (meters)')
    parser.add_argument('--gain', type=float, help='Sonar gain')
    parser.add_argument('--unidirectional', type=str, choices=['true', 'false'],
                        help='Unidirectional scan mode (true/false)')
    parser.add_argument('--step-multiplier', type=int,
                        help='Step multiplier for virtual step angle (1=0.5°, 2=1.0°, 4=2.0°)')
    return parser.parse_args(args)


def main():
    global running

    if len(sys.argv) < 2:
        print("Usage:")
        print("  test_control.py thruster <rpm> [rate_hz]")
        print("  test_control.py vbs <percentage> [rate_hz]")
        print("  test_control.py angle <yaw_rad> <pitch_rad> [rate_hz]")
        print("  test_control.py config <command> [value]")
        print("  test_control.py msis [options]")
        print("\nExamples:")
        print("  test_control.py thruster 500        # Send at 10 Hz (default)")
        print("  test_control.py thruster 500 20    # Send at 20 Hz")
        print("  test_control.py vbs 75")
        print("  test_control.py angle 0.1 -0.05")
        print("  test_control.py config get_status")
        print("\nMSIS Examples:")
        print("  test_control.py msis --get                                    # Get current config")
        print("  test_control.py msis --rotation-min -45 --rotation-max 45     # Set scan range")
        print("  test_control.py msis --range-max 80                           # Set detection range")
        print("  test_control.py msis --gain 1.5                               # Set gain")
        print("  test_control.py msis --step-multiplier 2                      # Set step multiplier (1.0° effective)")
        print("  test_control.py msis --unidirectional true                    # Set scan mode")
        print("\nCommands run continuously until Ctrl+C (except config and msis)")
        sys.exit(1)

    # Load configuration
    config_path = os.path.join(_SCRIPT_DIR, '..', 'config', 'test_control.jsonc')
    config = load_jsonc(config_path)
    actuators = config['actuators']
    services = config['services']

    cmd_type = sys.argv[1]

    if cmd_type == 'thruster':
        if len(sys.argv) < 3:
            print("Error: Missing RPM value")
            sys.exit(1)
        rpm = float(sys.argv[2])
        rate_hz = int(sys.argv[3]) if len(sys.argv) > 3 else 10
        send_thruster_continuous(actuators['thruster1'], rpm, rate_hz)

    elif cmd_type == 'vbs':
        if len(sys.argv) < 3:
            print("Error: Missing percentage value")
            sys.exit(1)
        percentage = float(sys.argv[2])
        rate_hz = int(sys.argv[3]) if len(sys.argv) > 3 else 10
        send_vbs_continuous(actuators['vbs'], percentage, rate_hz)

    elif cmd_type == 'angle':
        if len(sys.argv) < 4:
            print("Error: Missing yaw and pitch values")
            sys.exit(1)
        yaw = float(sys.argv[2])
        pitch = float(sys.argv[3])
        rate_hz = int(sys.argv[4]) if len(sys.argv) > 4 else 10
        send_thrust_angle_continuous(actuators['thrust_angle'], yaw, pitch, rate_hz)

    elif cmd_type == 'config':
        if len(sys.argv) < 3:
            print("Error: Missing config command")
            sys.exit(1)
        value = sys.argv[3] if len(sys.argv) > 3 else None
        send_config_command(services['config_server'], sys.argv[2], value)

    elif cmd_type == 'msis':
        config_addr = services['config_server']
        args = parse_msis_args(sys.argv[2:])
        if args.get:
            get_msis_config(config_addr)
        else:
            # Collect parameters to send
            unidirectional = None
            if args.unidirectional is not None:
                unidirectional = args.unidirectional.lower() == 'true'

            # Check if any parameter was provided
            if (args.rotation_min is None and args.rotation_max is None and
                args.range_min is None and args.range_max is None and
                args.gain is None and unidirectional is None and
                args.step_multiplier is None):
                print("Error: No MSIS parameters specified. Use --get to query current config.")
                print("Use --help for available options.")
                sys.exit(1)

            # Check rotation limits are provided together
            if (args.rotation_min is not None) != (args.rotation_max is not None):
                print("Error: Both --rotation-min and --rotation-max must be provided together")
                sys.exit(1)

            send_msis_config(
                config_addr,
                rotation_min=args.rotation_min,
                rotation_max=args.rotation_max,
                range_min=args.range_min,
                range_max=args.range_max,
                gain=args.gain,
                unidirectional=unidirectional,
                step_multiplier=args.step_multiplier
            )

    else:
        print(f"Unknown command type: {cmd_type}")
        sys.exit(1)

if __name__ == '__main__':
    main()
