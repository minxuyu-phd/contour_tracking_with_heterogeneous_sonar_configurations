import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from scripts import Publisher, Subscriber, Requester, Replier, Message


def test_pubsub():
    pub = Publisher()
    assert pub.init("tcp://127.0.0.1:15570")

    sub = Subscriber()
    assert sub.init("tcp://127.0.0.1:15570")
    time.sleep(0.1)

    msg = {"topic": "test", "value": 42}
    Message.add_timestamp(msg)
    assert pub.publish(msg)

    received = sub.receive(timeout_ms=2000)
    assert received is not None
    assert received["topic"] == "test"
    assert received["value"] == 42

    pub.close()
    sub.close()
    print("test_pubsub PASSED")


def test_reqrep():
    def handler(req):
        return {"result": req.get("x", 0) + req.get("y", 0)}

    rep = Replier()
    assert rep.init("tcp://127.0.0.1:15571")
    rep.set_handler(handler)
    rep.start()

    req = Requester()
    assert req.init("tcp://127.0.0.1:15571")
    time.sleep(0.1)

    reply = req.request({"x": 3, "y": 4}, timeout_ms=5000)
    assert reply is not None
    assert reply["result"] == 7

    req.close()
    rep.close()
    print("test_reqrep PASSED")


def test_async_subscriber():
    pub = Publisher()
    assert pub.init("tcp://127.0.0.1:15572")

    counter = {"count": 0}

    def callback(msg):
        counter["count"] += 1

    sub = Subscriber()
    assert sub.init("tcp://127.0.0.1:15572")
    sub.set_callback(callback)
    sub.start_async()
    time.sleep(0.1)

    for i in range(5):
        pub.publish({"seq": i})
        time.sleep(0.05)

    time.sleep(0.5)
    sub.stop_async()

    assert counter["count"] == 5, f"Expected 5, got {counter['count']}"

    pub.close()
    sub.close()
    print("test_async_subscriber PASSED")


def test_pubsub_subscriber_first():
    sub = Subscriber()
    assert sub.init("tcp://127.0.0.1:15573")

    time.sleep(0.05)

    pub = Publisher()
    assert pub.init("tcp://127.0.0.1:15573")
    time.sleep(0.1)

    received = None
    for _ in range(20):
        pub.publish({"data": "hello"})
        received = sub.receive(timeout_ms=200)
        if received is not None:
            break

    assert received is not None
    assert received["data"] == "hello"

    pub.close()
    sub.close()
    print("test_pubsub_subscriber_first PASSED")


def test_reqrep_client_first():
    req = Requester()
    assert req.init("tcp://127.0.0.1:15574")

    time.sleep(0.05)

    def handler(r):
        return {"echo": r.get("msg", "")}

    rep = Replier()
    assert rep.init("tcp://127.0.0.1:15574")
    rep.set_handler(handler)
    rep.start()
    time.sleep(0.1)

    reply = req.request({"msg": "ping"}, timeout_ms=5000)
    assert reply is not None
    assert reply["echo"] == "ping"

    req.close()
    rep.close()
    print("test_reqrep_client_first PASSED")


if __name__ == "__main__":
    tests = [
        test_pubsub,
        test_reqrep,
        test_async_subscriber,
        test_pubsub_subscriber_first,
        test_reqrep_client_first,
    ]
    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"{test.__name__} FAILED: {e}")
            failed += 1

    print(f"\nResults: {passed} passed, {failed} failed out of {len(tests)}")
