from __future__ import annotations

from typing import List, Tuple

from replay_core.constants import JOINT_NAMES
from replay_core.types import RuntimeSnapshot


def joint_rows(snapshot: RuntimeSnapshot) -> List[Tuple[str, float, float | None, float | None, float | None]]:
    rows = []
    mj_pos = None if snapshot.mujoco_state is None else snapshot.mujoco_state.positions
    rb_pos = None if snapshot.robot_state is None else snapshot.robot_state.positions
    rb_torques = snapshot.robot_torques
    for idx, name in enumerate(JOINT_NAMES):
        rows.append((
            name,
            float(snapshot.current_target[idx]),
            None if mj_pos is None else float(mj_pos[idx]),
            None if rb_pos is None else float(rb_pos[idx]),
            None if rb_torques is None else float(rb_torques[idx]),
        ))
    return rows
