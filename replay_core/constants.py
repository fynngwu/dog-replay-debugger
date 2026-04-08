from __future__ import annotations

import numpy as np

NUM_JOINTS = 12
JOINT_NAMES = [
    'LF_HipA', 'LR_HipA', 'RF_HipA', 'RR_HipA',
    'LF_HipF', 'LR_HipF', 'RF_HipF', 'RR_HipF',
    'LF_Knee', 'LR_Knee', 'RF_Knee', 'RR_Knee',
]

# Policy order -> MuJoCo sim joint order.
POLICY_TO_SIM = np.array([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64)
SIM_TO_POLICY = np.array([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64)

ROBOT_CMD_PORT = 47001
ROBOT_STATE_PORT = 47002
DEFAULT_FRAME_DT = 0.02
