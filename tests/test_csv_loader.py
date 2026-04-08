from __future__ import annotations

from pathlib import Path

from replay_core.csv_loader import load_replay_csv


def test_load_csv(tmp_path: Path):
    path = tmp_path / "sample.csv"
    path.write_text(
        "timestamp_ms,scaled_action_0,scaled_action_1,scaled_action_2,scaled_action_3,scaled_action_4,scaled_action_5,scaled_action_6,scaled_action_7,scaled_action_8,scaled_action_9,scaled_action_10,scaled_action_11\n"
        "0,0,0,0,0,0,0,0,0,0,0,0,0\n"
        "20,1,1,1,1,1,1,1,1,1,1,1,1\n",
        encoding="utf-8",
    )
    seq = load_replay_csv(path)
    assert seq.total_frames == 2
    assert seq.frames[1].target_rel[0] == 1.0
