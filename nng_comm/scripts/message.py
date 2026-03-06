import json
import time


class Message:
    @staticmethod
    def serialize(data: dict) -> bytes:
        return json.dumps(data).encode("utf-8")

    @staticmethod
    def deserialize(data: bytes) -> dict:
        return json.loads(data.decode("utf-8"))

    @staticmethod
    def add_timestamp(msg: dict) -> None:
        msg["timestamp"] = time.time()

    @staticmethod
    def get_timestamp() -> float:
        return time.time()
