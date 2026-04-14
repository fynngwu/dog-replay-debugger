from __future__ import annotations

import json
import socket

import protocol


class SMClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 48001):
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None

    def connect(self) -> tuple[bool, str]:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(3.0)
            self._sock.connect((self._host, self._port))
            self._sock.settimeout(5.0)
            return True, "connected"
        except Exception as e:
            self._sock = None
            return False, str(e)

    def disconnect(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def _send_recv(self, cmd: str) -> dict:
        if not self._sock:
            return {"ok": False, "msg": "not connected"}
        try:
            self._sock.sendall((cmd + "\n").encode())
            data = b""
            while True:
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
                data += chunk
                if b"\n" in data:
                    break
            return json.loads(data.decode().strip())
        except Exception as e:
            return {"ok": False, "msg": str(e)}

    def request_mode(self, mode: str) -> dict:
        return self._send_recv(protocol.make_request_mode(mode))

    def send_target(self, joints: list[float]) -> dict:
        return self._send_recv(protocol.make_target(joints))

    def get_mode(self) -> dict:
        return self._send_recv(protocol.make_get_mode())

    def get_joints(self) -> dict:
        return self._send_recv(protocol.make_get_joints())

    def get_imu(self) -> dict:
        return self._send_recv(protocol.make_get_imu())
