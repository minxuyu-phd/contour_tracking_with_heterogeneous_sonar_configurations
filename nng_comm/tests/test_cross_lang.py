#!/usr/bin/env python3
"""Cross-language test: Python side.

Usage:
    python test_cross_lang.py <mode> <address>
    mode: pub | sub | rep | req
    address: e.g. tcp://127.0.0.1:15580
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from scripts import Publisher, Subscriber, Requester, Replier, Message


def run_publisher(address: str):
    pub = Publisher()
    assert pub.init(address), "Publisher init failed"
    print(f"[Python Publisher] listening on {address}")

    # 等待订阅端连接
    time.sleep(0.5)

    for i in range(20):
        msg = {"source": "python", "type": "cross_lang_test", "seq": i, "value": 42}
        Message.add_timestamp(msg)
        pub.publish(msg)
        time.sleep(0.1)

    # 等待对端处理完毕
    time.sleep(0.5)
    pub.close()
    print("[Python Publisher] done")


def run_subscriber(address: str):
    sub = Subscriber()
    assert sub.init(address), "Subscriber init failed"
    print(f"[Python Subscriber] dialing {address}")

    received = None
    for _ in range(30):
        received = sub.receive(timeout_ms=500)
        if received is not None:
            break

    assert received is not None, "No message received"
    assert received["source"] == "cpp", f"Expected source 'cpp', got '{received['source']}'"
    assert received["type"] == "cross_lang_test"
    print(f"[Python Subscriber] received: {received}")

    sub.close()
    print("[Python Subscriber] PASSED")


def run_replier(address: str):
    def handler(req):
        print(f"[Python Replier] received: {req}")
        if req.get("command") == "add":
            result = req.get("a", 0) + req.get("b", 0)
            return {"status": "ok", "result": result, "server": "python"}
        return {"status": "error", "message": "unknown command"}

    rep = Replier()
    assert rep.init(address), "Replier init failed"
    rep.set_handler(handler)
    rep.start()
    print(f"[Python Replier] listening on {address}")

    # 运行一段时间等待请求
    time.sleep(5)

    rep.stop()
    rep.close()
    print("[Python Replier] done")


def run_requester(address: str):
    # 等待服务端启动
    time.sleep(0.5)

    req = Requester()
    assert req.init(address), "Requester init failed"
    print(f"[Python Requester] dialing {address}")

    time.sleep(0.2)

    request = {"command": "add", "a": 100, "b": 200}
    reply = req.request(request, timeout_ms=5000)
    print(f"[Python Requester] reply: {reply}")

    assert reply is not None, "No reply received"
    assert reply["status"] == "ok"
    assert reply["result"] == 300
    assert reply["server"] == "cpp"

    req.close()
    print("[Python Requester] PASSED")


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)

    mode = sys.argv[1]
    address = sys.argv[2]

    runners = {
        "pub": run_publisher,
        "sub": run_subscriber,
        "rep": run_replier,
        "req": run_requester,
    }

    if mode not in runners:
        print(f"Unknown mode: {mode}")
        print(__doc__)
        sys.exit(1)

    runners[mode](address)


if __name__ == "__main__":
    main()
