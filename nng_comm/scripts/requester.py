from typing import Optional

import pynng
from .message import Message


class Requester:
    def __init__(self):
        self._socket = None
        self._initialized = False

    def init(self, server_address: str) -> bool:
        try:
            self._socket = pynng.Req0()
            self._socket.dial(server_address, block=False)
            self._initialized = True
            return True
        except Exception as e:
            print(f"Requester init failed: {e}")
            self._initialized = False
            return False

    def request(self, req: dict, timeout_ms: int = 5000) -> Optional[dict]:
        if not self._initialized:
            return None
        try:
            self._socket.send_timeout = timeout_ms
            self._socket.recv_timeout = timeout_ms
            self._socket.send(Message.serialize(req))
            data = self._socket.recv()
            return Message.deserialize(data)
        except pynng.Timeout:
            return None
        except Exception as e:
            print(f"Requester request failed: {e}")
            return None

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
