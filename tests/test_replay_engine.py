from __future__ import annotations

from pathlib import Path

from replay_core.joint_limits import clamp_relative_targets
from replay_core.replay_engine import ReplayEngine


def _write_csv(path: Path):
    cols = ",".join([f"scaled_action_{i}" for i in range(12)])
    path.write_text(
        "timestamp_ms," + cols + "\n"
        + "0," + ",".join(["0"] * 12) + "\n"
        + "20," + ",".join(["1"] * 12) + "\n"
        + "40," + ",".join(["2"] * 12) + "\n",
        encoding="utf-8",
    )


def test_step_prev_seek(tmp_path: Path):
    csv_path = tmp_path / "sample.csv"
    _write_csv(csv_path)
    engine = ReplayEngine()
    try:
        assert engine.load_csv(str(csv_path))
        assert engine.get_snapshot().cursor == 0
        engine.step()
        assert engine.get_snapshot().cursor == 1
        engine.prev()
        assert engine.get_snapshot().cursor == 0
        engine.seek(2)
        snap = engine.get_snapshot()
        assert snap.cursor == 2
        assert snap.current_target_raw[0] == 2.0
        assert snap.current_target[0] < snap.current_target_raw[0]
    finally:
        engine.close()


def test_replay_uses_daemon_equivalent_clamped_target(tmp_path: Path):
    csv_path = tmp_path / "sample.csv"
    cols = ",".join([f"scaled_action_{i}" for i in range(12)])
    row = ["0"] * 12
    row[8] = "1.0"
    csv_path.write_text(
        "timestamp_ms," + cols + "\n" + "0," + ",".join(row) + "\n",
        encoding="utf-8",
    )
    engine = ReplayEngine()
    try:
        assert engine.load_csv(str(csv_path))
        snap = engine.get_snapshot()
        expected = clamp_relative_targets([0.0] * 8 + [1.0, 0.0, 0.0, 0.0])
        assert snap.current_target_raw[8] == 1.0
        assert abs(snap.current_target[8] - expected[8]) < 1e-6
    finally:
        engine.close()
