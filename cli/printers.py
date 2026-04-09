from __future__ import annotations

from typing import Iterable

import numpy as np

from replay_core.constants import JOINT_NAMES
from replay_core.types import RuntimeSnapshot


def fmt_array(values: np.ndarray | None, precision: int = 3) -> str:
    if values is None:
        return 'None'
    return '[' + ', '.join(f'{float(v):.{precision}f}' for v in values.tolist()) + ']'


def print_status(snapshot: RuntimeSnapshot) -> None:
    print(f'CSV: {snapshot.csv_path}')
    print(f'Frames: {snapshot.cursor}/{max(0, snapshot.total_frames - 1)}  loaded={snapshot.sequence_loaded}  playing={snapshot.playing}  speed={snapshot.playback_speed:.2f}x')
    print(f'Robot: connected={snapshot.robot_connected} tx={snapshot.robot_tx_hz:.1f}Hz rx={snapshot.robot_rx_hz:.1f}Hz age={snapshot.robot_state_age_s}')
    print(f'MuJoCo: loaded={snapshot.mujoco_loaded} viewer={snapshot.mujoco_viewer_running} apply={snapshot.mujoco_apply_hz:.1f}Hz age={snapshot.mujoco_state_age_s}')
    print(f'Target raw: {fmt_array(snapshot.current_target_raw)}')
    print(f'Target clamped: {fmt_array(snapshot.current_target)}')
    if snapshot.mujoco_state is not None:
        print(f'MuJoCo pos: {fmt_array(snapshot.mujoco_state.positions)}')
        print(f'MuJoCo err: {fmt_array(snapshot.mujoco_error)}')
    if snapshot.robot_state is not None:
        print(f'Robot pos:  {fmt_array(snapshot.robot_state.positions)}')
        print(f'Robot err:  {fmt_array(snapshot.robot_error)}')


def print_joint_table(snapshot: RuntimeSnapshot) -> None:
    print('joint          target    mujoco     robot   mj_err  robot_err')
    print('-' * 66)
    mujoco = None if snapshot.mujoco_state is None else snapshot.mujoco_state.positions
    robot = None if snapshot.robot_state is None else snapshot.robot_state.positions
    mj_err = snapshot.mujoco_error
    rb_err = snapshot.robot_error
    for i, name in enumerate(JOINT_NAMES):
        print(
            f'{name:<12} '
            f'{snapshot.current_target[i]:>7.3f} '
            f'{"-" if mujoco is None else f"{mujoco[i]:7.3f}"} '
            f'{"-" if robot is None else f"{robot[i]:7.3f}"} '
            f'{"-" if mj_err is None else f"{mj_err[i]:7.3f}"} '
            f'{"-" if rb_err is None else f"{rb_err[i]:7.3f}"}'
        )


def print_logs(lines: Iterable[str], tail: int = 20) -> None:
    seq = list(lines)[-tail:]
    for line in seq:
        print(line)
