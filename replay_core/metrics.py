from __future__ import annotations

import time
from collections import deque


class RateTracker:
    def __init__(self, window_sec: float = 2.0):
        self.window_sec = float(window_sec)
        self._events = deque()

    def tick(self) -> None:
        now = time.monotonic()
        self._events.append(now)
        self._trim(now)

    def value(self) -> float:
        now = time.monotonic()
        self._trim(now)
        if len(self._events) < 2:
            return 0.0
        dt = self._events[-1] - self._events[0]
        if dt <= 1e-9:
            return 0.0
        return (len(self._events) - 1) / dt

    def _trim(self, now: float) -> None:
        cutoff = now - self.window_sec
        while self._events and self._events[0] < cutoff:
            self._events.popleft()
