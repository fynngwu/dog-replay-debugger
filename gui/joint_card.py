"""Joint card component for displaying individual joint data."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QGridLayout, QLabel, QWidget
from PySide6.QtCore import Signal


class JointCard(QFrame):
    """A visual card displaying all data for a single joint."""

    clicked = Signal(str)  # Signal when card is clicked, sends joint name

    # Define field configuration
    FIELD_CONFIG = [
        ("Target", 1, "val"),
        ("MuJoCo", 2, "val"),
        ("Robot", 3, "val"),
        ("Err_M", 4, "val_error"),
        ("Err_R", 5, "val_error"),
    ]

    # Error thresholds
    ERROR_THRESHOLD_HIGH = 0.1
    ERROR_THRESHOLD_WARN = 0.05

    def __init__(self, name: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._name = name
        self._selected = False

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setObjectName("JointCard")
        self.setStyleSheet(self._get_card_style())

        self._setup_ui()

    def _get_card_style(self) -> str:
        """Get the card stylesheet."""
        from gui.styles import JOINT_CARD_STYLE
        return JOINT_CARD_STYLE

    def _setup_ui(self) -> None:
        """Setup the card UI layout."""
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(2)

        # Joint name header
        name_lbl = QLabel(self._name)
        name_lbl.setObjectName("name")
        layout.addWidget(name_lbl, 0, 0, 1, 2)

        # Setup field rows
        self.labels = {}
        for text, row, obj_name in self.FIELD_CONFIG:
            # Field label
            label = QLabel(text)
            label.setObjectName("label")
            layout.addWidget(label, row, 0)

            # Value label
            val_lbl = QLabel("-")
            val_lbl.setObjectName(obj_name)
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
            layout.addWidget(val_lbl, row, 1)
            self.labels[text] = val_lbl

        # Make card clickable
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def mousePressEvent(self, event) -> None:  # noqa: N802
        """Handle mouse press event for card selection."""
        self.clicked.emit(self._name)
        super().mousePressEvent(event)

    def update_data(
        self,
        target: float,
        mujoco: float | None,
        robot: float | None,
        err_m: float | None,
        err_r: float | None,
    ) -> None:
        """Update the card with new data."""
        # Update target
        self.labels["Target"].setText(f"{target:.4f}")

        # Update MuJoCo position
        self.labels["MuJoCo"].setText(f"{mujoco:.4f}" if mujoco is not None else "--")

        # Update robot position
        self.labels["Robot"].setText(f"{robot:.4f}" if robot is not None else "--")

        # Update MuJoCo error with color coding
        if err_m is not None:
            self.labels["Err_M"].setText(f"{err_m:.4f}")
            abs_err = abs(err_m)
            if abs_err > self.ERROR_THRESHOLD_HIGH:
                self.labels["Err_M"].setStyleSheet("color: #ff5252; font-family: 'Consolas'; font-size: 12px; padding: 2px;")
            elif abs_err > self.ERROR_THRESHOLD_WARN:
                self.labels["Err_M"].setStyleSheet("color: #ffd740; font-family: 'Consolas'; font-size: 12px; padding: 2px;")
            else:
                self.labels["Err_M"].setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 12px; padding: 2px;")
        else:
            self.labels["Err_M"].setText("--")
            self.labels["Err_M"].setStyleSheet("color: #606070; font-family: 'Consolas'; font-size: 12px; padding: 2px;")

        # Update robot error with color coding
        if err_r is not None:
            self.labels["Err_R"].setText(f"{err_r:.4f}")
            abs_err = abs(err_r)
            if abs_err > self.ERROR_THRESHOLD_HIGH:
                self.labels["Err_R"].setStyleSheet("color: #ff5252; font-family: 'Consolas'; font-size: 12px; padding: 2px;")
            elif abs_err > self.ERROR_THRESHOLD_WARN:
                self.labels["Err_R"].setStyleSheet("color: #ffd740; font-family: 'Consolas'; font-size: 12px; padding: 2px;")
            else:
                self.labels["Err_R"].setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 12px; padding: 2px;")
        else:
            self.labels["Err_R"].setText("--")
            self.labels["Err_R"].setStyleSheet("color: #606070; font-family: 'Consolas'; font-size: 12px; padding: 2px;")

    def set_selected(self, selected: bool) -> None:
        """Set the card's selected state."""
        if self._selected == selected:
            return

        self._selected = selected
        self.setProperty("selected", "true" if selected else "false")
        self.style().unpolish(self)
        self.style().polish(self)

    @property
    def name(self) -> str:
        """Get the joint name."""
        return self._name
