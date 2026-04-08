from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable, List, Tuple

import numpy as np

from replay_core.constants import DEFAULT_FRAME_DT, NUM_JOINTS
from replay_core.types import ReplayFrame, ReplaySequence


class CsvLoadError(RuntimeError):
    pass


TARGET_COLUMN_CANDIDATES = (
    ('target_rel', 'target_rel_{}'),
    ('scaled_action', 'scaled_action_{}'),
)

TIME_COLUMN_CANDIDATES = (
    'timestamp_ms',
    'sim_time',
)


def _detect_target_columns(fieldnames: Iterable[str]) -> Tuple[str, str]:
    names = set(fieldnames)
    for label, pattern in TARGET_COLUMN_CANDIDATES:
        if all(pattern.format(i) in names for i in range(NUM_JOINTS)):
            return label, pattern
    raise CsvLoadError(
        'CSV must contain target_rel_0..11 or scaled_action_0..11; target_q_* is intentionally not used here'
    )


def _detect_time_column(fieldnames: Iterable[str]) -> str:
    names = set(fieldnames)
    for candidate in TIME_COLUMN_CANDIDATES:
        if candidate in names:
            return candidate
    return 'row_index'


def load_replay_csv(path: str | Path, default_dt: float = DEFAULT_FRAME_DT) -> ReplaySequence:
    csv_path = Path(path).expanduser().resolve()
    if not csv_path.exists():
        raise CsvLoadError(f'CSV not found: {csv_path}')

    frames: List[ReplayFrame] = []
    with csv_path.open('r', encoding='utf-8-sig', newline='') as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise CsvLoadError('CSV has no header')
        target_label, target_pattern = _detect_target_columns(reader.fieldnames)
        time_label = _detect_time_column(reader.fieldnames)

        for idx, row in enumerate(reader):
            if time_label == 'timestamp_ms':
                time_sec = float(row[time_label]) / 1000.0
            elif time_label == 'sim_time':
                time_sec = float(row[time_label])
            else:
                time_sec = idx * float(default_dt)
            target_rel = np.array([float(row[target_pattern.format(i)]) for i in range(NUM_JOINTS)], dtype=np.float64)
            frames.append(ReplayFrame(index=idx, time_sec=time_sec, target_rel=target_rel))

    if not frames:
        raise CsvLoadError(f'CSV contains no replay frames: {csv_path}')

    if len(frames) >= 2:
        dts = [max(1e-6, frames[i + 1].time_sec - frames[i].time_sec) for i in range(len(frames) - 1)]
        estimated_dt = float(sum(dts) / len(dts))
    else:
        estimated_dt = float(default_dt)

    return ReplaySequence(
        csv_path=csv_path,
        frames=frames,
        estimated_dt=estimated_dt,
        target_columns=target_label,
        time_columns=time_label,
    )
