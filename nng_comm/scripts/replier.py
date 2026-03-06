import threading
from typing import Callable

import pynng
from .message import Message


class Replier:
    def __init__(self):
        self._socket = None
        self._initialized = False
        self._handler = None
        self._running = False
        self._thread = None

    def init(self, bind_address: str) -> bool:
        try:
            self._socket = pynng.Rep0()
            self._socket.listen(bind_address)
            self._initialized = True
            return True
        except Exception as e:
            print(f"Replier init failed: {e}")
            self._initialized = False
            return False

    def set_handler(self, handler: Callable[[dict], dict]):
        self._handler = handler

    def start(self):
        if not self._initialized or self._running or self._handler is None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._service_loop, daemon=True)
        self._thread.start()

    def _service_loop(self):
        while self._running:
            try:
                self._socket.recv_timeout = 100
                data = self._socket.recv()
                req = Message.deserialize(data)
                reply = self._handler(req)
                self._socket.send(Message.serialize(reply))
            except pynng.Timeout:
                continue
            except Exception:
                if self._running:
                    continue
                break

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def close(self):
        self.stop()
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
