from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np


@dataclass(slots=True)
class ReplayFrame:
    index: int
    time_sec: float
    target_rel: np.ndarray


@dataclass(slots=True)
class ReplaySequence:
    csv_path: Path
    frames: List[ReplayFrame]
    estimated_dt: float
    target_columns: str
    time_columns: str

    @property
    def total_frames(self) -> int:
        return len(self.frames)


@dataclass(slots=True)
class JointState:
    positions: np.ndarray
    velocities: np.ndarray
    timestamp_sec: float


@dataclass(slots=True)
class RuntimeSnapshot:
    csv_path: Optional[str]
    sequence_loaded: bool
    total_frames: int
    cursor: int
    playing: bool
    playback_speed: float
    current_target_raw: np.ndarray
    current_target: np.ndarray
    robot_connected: bool
    robot_tx_hz: float
    robot_rx_hz: float
    robot_state: Optional[JointState]
    robot_state_age_s: Optional[float]
    mujoco_loaded: bool
    mujoco_viewer_running: bool
    mujoco_apply_hz: float
    mujoco_state: Optional[JointState]
    mujoco_state_age_s: Optional[float]
    log_lines: List[str] = field(default_factory=list)

    @property
    def robot_error(self) -> Optional[np.ndarray]:
        if self.robot_state is None:
            return None
        return self.robot_state.positions - self.current_target

    @property
    def mujoco_error(self) -> Optional[np.ndarray]:
        if self.mujoco_state is None:
            return None
        return self.mujoco_state.positions - self.current_target
