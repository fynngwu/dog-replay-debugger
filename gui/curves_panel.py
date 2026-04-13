from __future__ import annotations

from collections import deque

from replay_core.constants import JOINT_NAMES

try:
    import pyqtgraph as pg
    from PySide6.QtWidgets import QComboBox, QVBoxLayout, QWidget, QLabel
except Exception:  # pragma: no cover
    pg = None
    from PySide6.QtWidgets import QLabel as QWidget  # type: ignore


class CurvesPanel(QWidget):
    """Panel for displaying joint position history curves."""

    def __init__(self):
        if pg is None:
            super().__init__('pyqtgraph is not installed')
            self.joint_combo = None
            return
        super().__init__()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Header
        header_layout = QVBoxLayout()
        title = QLabel("Joint History")
        title.setStyleSheet("color: #b0b0c0; font-weight: 600; font-size: 13px; padding: 6px;")
        header_layout.addWidget(title)

        # Combo box for joint selection
        self.joint_combo = QComboBox()
        self.joint_combo.addItems(JOINT_NAMES)
        self.joint_combo.setStyleSheet("""
            QComboBox {
                background-color: #2d2d3a;
                border: 1px solid #3d3d4d;
                border-radius: 4px;
                padding: 4px 8px;
                color: #e0e0e0;
            }
        """)
        header_layout.addWidget(self.joint_combo)

        layout.addLayout(header_layout)

        # Plot widget
        self.plot = pg.PlotWidget()
        self.plot.setBackground('#1e1e26')
        self.plot.showGrid(x=True, y=True, alpha=0.3)

        # Add legend
        self.plot.addLegend()

        # Create curves
        pen_target = pg.mkPen(color='#00e5ff', width=2)
        pen_mujoco = pg.mkPen(color='#69f0ae', width=2)
        pen_robot = pg.mkPen(color='#ffd740', width=2)
        pen_torque = pg.mkPen(color='#ff5252', width=2)

        self.curve_target = self.plot.plot(name='target', pen=pen_target)
        self.curve_mujoco = self.plot.plot(name='mujoco', pen=pen_mujoco)
        self.curve_robot = self.plot.plot(name='robot', pen=pen_robot)
        self.curve_torque = self.plot.plot(name='torque', pen=pen_torque)

        layout.addWidget(self.plot)

        # Data buffers
        self.x = deque(maxlen=400)
        self.target = deque(maxlen=400)
        self.mujoco = deque(maxlen=400)
        self.robot = deque(maxlen=400)
        self.torque = deque(maxlen=400)
        self._counter = 0

    def push(self, snapshot) -> None:  # noqa: ANN001
        """Push new snapshot data to the curves."""
        if pg is None or self.joint_combo is None:
            return

        idx = self.joint_combo.currentIndex()
        self._counter += 1
        self.x.append(self._counter)
        self.target.append(float(snapshot.current_target[idx]))
        self.mujoco.append(
            None if snapshot.mujoco_state is None else float(snapshot.mujoco_state.positions[idx])
        )
        self.robot.append(
            None if snapshot.robot_state is None else float(snapshot.robot_state.positions[idx])
        )
        self.torque.append(
            None if snapshot.robot_torques is None else float(snapshot.robot_torques[idx])
        )

        x = list(self.x)
        self.curve_target.setData(x, [0.0 if v is None else v for v in self.target])
        self.curve_mujoco.setData(x, [0.0 if v is None else v for v in self.mujoco])
        self.curve_robot.setData(x, [0.0 if v is None else v for v in self.robot])
        self.curve_torque.setData(x, [0.0 if v is None else v for v in self.torque])

    def set_joint(self, joint_name: str) -> None:
        """Set the current joint to display."""
        if self.joint_combo and joint_name in JOINT_NAMES:
            idx = JOINT_NAMES.index(joint_name)
            self.joint_combo.setCurrentIndex(idx)
            # Clear data buffers when switching joints
            self.x.clear()
            self.target.clear()
            self.mujoco.clear()
            self.robot.clear()
            self.torque.clear()
            self._counter = 0
