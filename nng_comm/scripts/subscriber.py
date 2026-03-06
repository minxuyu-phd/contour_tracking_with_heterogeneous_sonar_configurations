import threading
from typing import Callable, Optional

import pynng
from .message import Message


class Subscriber:
    def __init__(self):
        self._socket = None
        self._initialized = False
        self._callback = None
        self._running = False
        self._thread = None

    def init(self, address: str) -> bool:
        try:
            self._socket = pynng.Sub0()
            self._socket.subscribe(b"")
            self._socket.dial(address, block=False)
            self._initialized = True
            return True
        except Exception as e:
            print(f"Subscriber init failed: {e}")
            self._initialized = False
            return False

    def receive(self, timeout_ms: int = -1) -> Optional[dict]:
        if not self._initialized:
            return None
        try:
            if timeout_ms >= 0:
                self._socket.recv_timeout = timeout_ms
            else:
                self._socket.recv_timeout = None
            data = self._socket.recv()
            return Message.deserialize(data)
        except pynng.Timeout:
            return None
        except Exception as e:
            print(f"Subscriber receive failed: {e}")
            return None

    def set_callback(self, callback: Callable[[dict], None]):
        self._callback = callback

    def start_async(self):
        if not self._initialized or self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def _receive_loop(self):
        while self._running:
            try:
                self._socket.recv_timeout = 100
                data = self._socket.recv()
                if self._callback:
                    self._callback(Message.deserialize(data))
            except pynng.Timeout:
                continue
            except Exception:
                if self._running:
                    continue
                break

    def stop_async(self):
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def close(self):
        self.stop_async()
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
