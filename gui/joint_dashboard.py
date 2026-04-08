"""Joint dashboard component for displaying all joints in a grid layout."""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QFrame, QGridLayout, QLabel, QVBoxLayout, QWidget

from gui.joint_card import JointCard
from replay_core.constants import JOINT_NAMES


class JointDashboard(QWidget):
    """Dashboard displaying all 12 joints in an organized grid layout."""

    joint_selected = Signal(str)  # Signal when a joint card is selected

    # Joint grouping by leg
    LEG_GROUPS = {
        "FL": ["LF_HipA", "LF_HipF", "LF_Knee"],
        "FR": ["RF_HipA", "RF_HipF", "RF_Knee"],
        "RL": ["LR_HipA", "LR_HipF", "LR_Knee"],
        "RR": ["RR_HipA", "RR_HipF", "RR_Knee"],
    }

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._cards: dict[str, JointCard] = {}
        self._selected_joint: str | None = None

        self.setStyleSheet(self._get_dashboard_style())
        self._setup_ui()

    def _get_dashboard_style(self) -> str:
        """Get the dashboard stylesheet."""
        from gui.styles import DASHBOARD_STYLE
        return DASHBOARD_STYLE

    def _setup_ui(self) -> None:
        """Setup the dashboard UI layout."""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(12)

        # Title
        title = QLabel("Joint Real-time Monitor")
        title.setObjectName("dashboard_title")
        main_layout.addWidget(title)

        # Grid layout for leg groups
        grid = QGridLayout()
        grid.setSpacing(16)

        # Create leg group frames in a 2x2 grid: FL|RL, FR|RR
        leg_positions = [("FL", 0, 0), ("RL", 0, 1), ("FR", 1, 0), ("RR", 1, 1)]

        for leg_name, row, col in leg_positions:
            leg_frame = self._create_leg_frame(leg_name)
            grid.addWidget(leg_frame, row, col)

        main_layout.addLayout(grid)

    def _create_leg_frame(self, leg_name: str) -> QFrame:
        """Create a frame for a leg group."""
        frame = QFrame()
        frame.setObjectName("leg_frame")

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Leg label
        label = QLabel(f"{leg_name} Leg")
        label.setObjectName("leg_group")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(label)

        # Create cards for this leg
        leg_joints = self.LEG_GROUPS[leg_name]
        for joint_name in leg_joints:
            card = JointCard(joint_name)
            card.clicked.connect(self._on_card_clicked)
            self._cards[joint_name] = card
            layout.addWidget(card)

        layout.addStretch()
        return frame

    def _on_card_clicked(self, joint_name: str) -> None:
        """Handle card click event."""
        # Update selection
        if self._selected_joint:
            self._cards[self._selected_joint].set_selected(False)

        self._selected_joint = joint_name
        self._cards[joint_name].set_selected(True)

        # Emit signal
        self.joint_selected.emit(joint_name)

    def update_all(self, snapshot) -> None:  # noqa: ANN001
        """Update all joint cards with snapshot data."""
        if snapshot is None:
            return

        mj_pos = snapshot.mujoco_state.positions if snapshot.mujoco_state else [None] * 12
        rb_pos = snapshot.robot_state.positions if snapshot.robot_state else [None] * 12
        mj_err = snapshot.mujoco_error if snapshot.mujoco_error is not None else [None] * 12
        rb_err = snapshot.robot_error if snapshot.robot_error is not None else [None] * 12

        for idx, name in enumerate(JOINT_NAMES):
            if name in self._cards:
                self._cards[name].update_data(
                    target=float(snapshot.current_target[idx]),
                    mujoco=mj_pos[idx] if mj_pos[idx] is not None else None,
                    robot=rb_pos[idx] if rb_pos[idx] is not None else None,
                    err_m=mj_err[idx] if mj_err[idx] is not None else None,
                    err_r=rb_err[idx] if rb_err[idx] is not None else None,
                )

    def set_selected_joint(self, joint_name: str | None) -> None:
        """Set the selected joint programmatically."""
        if self._selected_joint:
            self._cards[self._selected_joint].set_selected(False)

        self._selected_joint = joint_name

        if joint_name and joint_name in self._cards:
            self._cards[joint_name].set_selected(True)

    @property
    def selected_joint(self) -> str | None:
        """Get the currently selected joint name."""
        return self._selected_joint
