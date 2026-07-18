"""Non-blocking UDP receiver. Always returns the newest packet in the queue."""
from __future__ import annotations

import json
import socket
from typing import Optional


class UdpReceiver:
    def __init__(self, port: int, bind_addr: str = "0.0.0.0"):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_addr, port))
        self._sock.setblocking(False)

    def poll(self) -> Optional[dict]:
        """Drain socket, return newest parseable packet or None."""
        latest = None
        while True:
            try:
                data, _ = self._sock.recvfrom(2048)
            except BlockingIOError:
                break
            try:
                latest = json.loads(data.decode("utf-8"))
            except Exception:
                continue
        return latest

    def close(self):
        try:
            self._sock.close()
        except Exception:
            pass
