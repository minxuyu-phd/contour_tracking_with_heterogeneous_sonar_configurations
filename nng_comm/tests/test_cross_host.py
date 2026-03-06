#!/usr/bin/env python3
"""Cross-host test: run one mode per machine (or terminal).

Usage:
    python test_cross_host.py --mode <MODE> --address <IP> [--port PORT]

    --mode:    pub | sub | rep | req  (required)
    --address: local IP for pub/rep (listen), or remote IP for sub/req (dial)
               accepts raw IP or full tcp://... URL
    --port:    port number (default: 15590)
"""
import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from scripts import Publisher, Subscriber, Requester, Replier, Message


def build_address(ip: str, port: int) -> str:
    if ip.startswith("tcp://"):
        return ip
    return f"tcp://{ip}:{port}"


def run_publisher(address: str):
    pub = Publisher()
    assert pub.init(address), "Publisher init failed"
    print(f"[pub] listening on {address}  (Ctrl+C to stop)")
    try:
        while True:
            msg = {}
            Message.add_timestamp(msg)
            pub.publish(msg)
            print(f"[pub] sent timestamp={msg.get('timestamp')}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[pub] stopping")
    finally:
        pub.close()


def run_subscriber(address: str):
    sub = Subscriber()
    assert sub.init(address), "Subscriber init failed"
    print(f"[sub] dialing {address}  (Ctrl+C to stop)")

    def callback(msg):
        print(f"[sub] received timestamp={msg.get('timestamp')}")

    sub.set_callback(callback)
    sub.start_async()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[sub] stopping")
    finally:
        sub.stop_async()
        sub.close()


def run_replier(address: str):
    def handler(req):
        print(f"[rep] received timestamp={req.get('timestamp')}")
        reply = {}
        Message.add_timestamp(reply)
        return reply

    rep = Replier()
    assert rep.init(address), "Replier init failed"
    rep.set_handler(handler)
    rep.start()
    print(f"[rep] listening on {address}  (Ctrl+C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[rep] stopping")
    finally:
        rep.stop()
        rep.close()


def run_requester(address: str):
    req = Requester()
    assert req.init(address), "Requester init failed"
    print(f"[req] dialing {address}  (Ctrl+C to stop)")
    try:
        while True:
            request = {}
            Message.add_timestamp(request)
            reply = req.request(request, timeout_ms=5000)
            print(f"[req] reply={reply}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[req] stopping")
    finally:
        req.close()


def main():
    parser = argparse.ArgumentParser(
        description="Cross-host NNG communication test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=["pub", "sub", "rep", "req"],
        help="communication role",
    )
    parser.add_argument(
        "--address",
        required=True,
        help="IP address or full tcp://IP:PORT URL",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=15590,
        help="port number (default: 15590, ignored if --address contains tcp://)",
    )
    args = parser.parse_args()

    address = build_address(args.address, args.port)

    runners = {
        "pub": run_publisher,
        "sub": run_subscriber,
        "rep": run_replier,
        "req": run_requester,
    }
    runners[args.mode](address)


if __name__ == "__main__":
    main()
