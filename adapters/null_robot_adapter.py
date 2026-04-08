from __future__ import annotations


class NullRobotAdapter:
    def connect(self, *args, **kwargs) -> bool:
        return False

    def disconnect(self) -> None:
        return None

    def ping(self) -> bool:
        return False

    def init(self, duration_s: float = 2.5):
        return {'ok': False, 'msg': 'robot not connected'}

    def enable(self):
        return {'ok': False, 'msg': 'robot not connected'}

    def disable(self):
        return {'ok': False, 'msg': 'robot not connected'}
