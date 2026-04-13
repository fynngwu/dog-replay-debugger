from __future__ import annotations

import numpy as np

from replay_core.constants import NUM_JOINTS

# Mirrored from dog_fifo_backend/daemon/motor_config.hpp so the GUI preview and
# manual joint editor use the same joint semantics as the new backend.
KNEE_RATIO = 1.667
ACTION_SCALE = 0.25

JOINT_DIRECTION = np.array([
    -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0,
    +1.0, +1.0, +1.0, +1.0,
], dtype=np.float64)

HIP_A_OFFSET = 0.37
HIP_F_OFFSET = 0.13
KNEE_OFFSET = 1.06 * KNEE_RATIO

JOINT_OFFSETS = np.array([
    HIP_A_OFFSET, -HIP_A_OFFSET, -HIP_A_OFFSET, HIP_A_OFFSET,
    HIP_F_OFFSET, HIP_F_OFFSET, -HIP_F_OFFSET, -HIP_F_OFFSET,
    KNEE_OFFSET, KNEE_OFFSET, -KNEE_OFFSET, -KNEE_OFFSET,
], dtype=np.float64)

XML_MIN = np.array([
    -0.7853982, -0.7853982, -0.7853982, -0.7853982,
    -1.2217658, -1.2217305, -0.8726999, -0.8726999,
    -1.0217299 * KNEE_RATIO, -1.0217299 * KNEE_RATIO, -0.6, -0.6,
], dtype=np.float64)

XML_MAX = np.array([
    0.7853982, 0.7853982, 0.7853982, 0.7853982,
    0.8726683, 0.8726683, 1.2217342, 1.2217305,
    0.6, 0.6, 1.0217287 * KNEE_RATIO, 1.0217287 * KNEE_RATIO,
], dtype=np.float64)

LOWER_ABS = JOINT_OFFSETS + XML_MIN
UPPER_ABS = JOINT_OFFSETS + XML_MAX
KNEE_MASK = np.array([False] * 8 + [True] * 4, dtype=bool)


def _as_joint_vector(values: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.shape[0] != NUM_JOINTS:
        raise ValueError(f'Expected {NUM_JOINTS} joint values, got {arr.shape[0]}')
    return arr


def relative_to_absolute(rel_targets: np.ndarray | list[float] | tuple[float, ...], action_scale: float = 1.0) -> np.ndarray:
    rel = _as_joint_vector(rel_targets)
    eff = rel.copy()
    eff[KNEE_MASK] *= KNEE_RATIO
    return JOINT_DIRECTION * eff * float(action_scale) + JOINT_OFFSETS


def absolute_to_relative(abs_targets: np.ndarray | list[float] | tuple[float, ...], action_scale: float = 1.0) -> np.ndarray:
    if abs(float(action_scale)) < 1e-12:
        raise ValueError('action_scale must be non-zero')
    abs_arr = _as_joint_vector(abs_targets)
    rel = JOINT_DIRECTION * (abs_arr - JOINT_OFFSETS) / float(action_scale)
    rel[KNEE_MASK] /= KNEE_RATIO
    return rel


def clamp_absolute_targets(abs_targets: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
    arr = _as_joint_vector(abs_targets)
    return np.clip(arr, LOWER_ABS, UPPER_ABS)


def clamp_relative_targets(rel_targets: np.ndarray | list[float] | tuple[float, ...], action_scale: float = 1.0) -> np.ndarray:
    abs_targets = relative_to_absolute(rel_targets, action_scale=action_scale)
    abs_limited = clamp_absolute_targets(abs_targets)
    return absolute_to_relative(abs_limited, action_scale=action_scale)


def clamp_single_relative_target(joint_index: int, rel_target: float, action_scale: float = 1.0) -> float:
    if joint_index < 0 or joint_index >= NUM_JOINTS:
        raise IndexError(f'Invalid joint index: {joint_index}')
    values = np.zeros(NUM_JOINTS, dtype=np.float64)
    values[joint_index] = float(rel_target)
    return float(clamp_relative_targets(values, action_scale=action_scale)[joint_index])
