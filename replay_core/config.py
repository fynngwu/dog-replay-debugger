from __future__ import annotations

from dataclasses import dataclass

from replay_core.constants import DEFAULT_FRAME_DT, ROBOT_CMD_PORT, ROBOT_STATE_PORT


@dataclass(slots=True)
class EngineConfig:
    play_loop_sleep_s: float = 0.002
    robot_tx_period_s: float = 0.02
    robot_timeout_s: float = 3.0
    robot_cmd_port: int = ROBOT_CMD_PORT
    robot_state_port: int = ROBOT_STATE_PORT
    default_frame_dt: float = DEFAULT_FRAME_DT
