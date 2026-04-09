from __future__ import annotations

import numpy as np

from replay_core.joint_limits import clamp_relative_targets


def test_knee_limits_match_daemon_semantics() -> None:
    raw = np.zeros(12, dtype=np.float64)
    raw[8] = 1.0     # LF_Knee should clamp down to ~0.3599
    raw[10] = -1.0   # RF_Knee should clamp up to ~-0.3599
    limited = clamp_relative_targets(raw)
    assert limited[8] < raw[8]
    assert limited[10] > raw[10]
    assert abs(limited[8] - 0.6 / 1.667) < 1e-4
    assert abs(limited[10] + 0.6 / 1.667) < 1e-4
