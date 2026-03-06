import pynng
from .message import Message


class Publisher:
    def __init__(self):
        self._socket = None
        self._initialized = False

    def init(self, address: str) -> bool:
        try:
            self._socket = pynng.Pub0()
            self._socket.listen(address)
            self._initialized = True
            return True
        except Exception as e:
            print(f"Publisher init failed: {e}")
            self._initialized = False
            return False

    def publish(self, msg: dict) -> bool:
        if not self._initialized:
            return False
        try:
            self._socket.send(Message.serialize(msg))
            return True
        except Exception as e:
            print(f"Publisher publish failed: {e}")
            return False

    def close(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None
            self._initialized = False

    def is_initialized(self) -> bool:
        return self._initialized

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
