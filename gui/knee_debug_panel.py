from __future__ import annotations

from typing import List

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from gui.backend_state_panel import BackendStatePanel
from replay_core.constants import JOINT_NAMES
from replay_core.joint_limits import XML_MAX, XML_MIN, clamp_single_relative_target


class JointEditorRow(QWidget):
    send_requested = Signal(int)
    setzero_requested = Signal(int)

    def __init__(self, joint_idx: int, parent: QWidget | None = None):
        super().__init__(parent)
        self.joint_idx = joint_idx
        self._updating = False

        layout = QGridLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        self.name_label = QLabel(JOINT_NAMES[joint_idx])
        self.name_label.setMinimumWidth(90)
        layout.addWidget(self.name_label, 0, 0)

        slider_min = int(round(float(XML_MIN[joint_idx]) * 1000.0))
        slider_max = int(round(float(XML_MAX[joint_idx]) * 1000.0))
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(slider_min, slider_max)
        self.slider.valueChanged.connect(self._on_slider_changed)
        layout.addWidget(self.slider, 0, 1)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(float(XML_MIN[joint_idx]), float(XML_MAX[joint_idx]))
        self.spin.setDecimals(4)
        self.spin.setSingleStep(0.01)
        self.spin.valueChanged.connect(self._on_spin_changed)
        layout.addWidget(self.spin, 0, 2)

        self.preview_label = QLabel('clamped=0.0000')
        self.preview_label.setStyleSheet("font-family: 'Consolas';")
        layout.addWidget(self.preview_label, 0, 3)

        self.state_label = QLabel('state=-- torque=--')
        self.state_label.setStyleSheet("font-family: 'Consolas';")
        layout.addWidget(self.state_label, 0, 4)

        self.send_btn = QPushButton('Send')
        self.send_btn.clicked.connect(lambda: self.send_requested.emit(self.joint_idx))
        layout.addWidget(self.send_btn, 0, 5)

        self.setzero_btn = QPushButton('SetZero')
        self.setzero_btn.clicked.connect(lambda: self.setzero_requested.emit(self.joint_idx))
        layout.addWidget(self.setzero_btn, 0, 6)

        self.set_editor_value(0.0)

    def _on_slider_changed(self, value: int) -> None:
        if self._updating:
            return
        self._updating = True
        try:
            self.spin.setValue(float(value) / 1000.0)
        finally:
            self._updating = False
        self._update_preview()

    def _on_spin_changed(self, value: float) -> None:
        if self._updating:
            return
        self._updating = True
        try:
            self.slider.setValue(int(round(float(value) * 1000.0)))
        finally:
            self._updating = False
        self._update_preview()

    def _update_preview(self) -> None:
        clamped = clamp_single_relative_target(self.joint_idx, self.editor_value())
        self.preview_label.setText(f'clamped={clamped:.4f}')

    def editor_value(self) -> float:
        return float(self.spin.value())

    def set_editor_value(self, value: float) -> None:
        self._updating = True
        try:
            self.spin.setValue(float(value))
            self.slider.setValue(int(round(float(value) * 1000.0)))
        finally:
            self._updating = False
        self._update_preview()

    def update_stream(self, state_pos: float | None, torque: float | None) -> None:
        pos_text = '--' if state_pos is None else f'{state_pos:.4f}'
        tq_text = '--' if torque is None else f'{torque:.4f}'
        self.state_label.setText(f'state={pos_text} torque={tq_text}')


class KneeDebugPanel(QWidget):
    send_all_requested = Signal(list)
    send_single_requested = Signal(list)
    set_zero_requested = Signal(int)
    record_requested = Signal()
    stop_record_requested = Signal()

    def __init__(self):
        super().__init__()
        self._latest_target = [0.0] * len(JOINT_NAMES)
        self._latest_stream = [0.0] * len(JOINT_NAMES)
        self.rows: list[JointEditorRow] = []
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        actions = QGroupBox('Joint Debug (all 12 joints)')
        actions_layout = QVBoxLayout(actions)
        actions_layout.setContentsMargins(12, 12, 12, 12)
        actions_layout.setSpacing(8)

        desc = QLabel(
            'Editors are local until you click Send. This avoids flooding the new FIFO backend while dragging sliders. '
            'Use Load stream pose to copy the latest pushed joint positions into the editors.'
        )
        desc.setWordWrap(True)
        actions_layout.addWidget(desc)

        btn_row = QHBoxLayout()
        self.load_stream_btn = QPushButton('Load stream pose -> editors')
        self.load_target_btn = QPushButton('Load UI target -> editors')
        self.zero_editors_btn = QPushButton('Zero editors')
        self.send_all_btn = QPushButton('Send all joints')
        btn_row.addWidget(self.load_stream_btn)
        btn_row.addWidget(self.load_target_btn)
        btn_row.addWidget(self.zero_editors_btn)
        btn_row.addWidget(self.send_all_btn)
        actions_layout.addLayout(btn_row)

        record_row = QHBoxLayout()
        self.record_btn = QPushButton('Record')
        self.stop_record_btn = QPushButton('Stop Record')
        self.stop_record_btn.setEnabled(False)
        self.record_btn.clicked.connect(self.record_requested)
        self.stop_record_btn.clicked.connect(self.stop_record_requested)
        record_row.addWidget(self.record_btn)
        record_row.addWidget(self.stop_record_btn)
        actions_layout.addLayout(record_row)
        layout.addWidget(actions)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        body_layout = QVBoxLayout(body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(6)

        for joint_idx in range(len(JOINT_NAMES)):
            row = JointEditorRow(joint_idx)
            row.send_requested.connect(self._emit_single)
            row.setzero_requested.connect(self.set_zero_requested)
            self.rows.append(row)
            body_layout.addWidget(row)
        body_layout.addStretch(1)
        scroll.setWidget(body)
        layout.addWidget(scroll, stretch=1)

        self.state_panel = BackendStatePanel('Backend Stream State (Joint Debug tab)')
        layout.addWidget(self.state_panel)

        self.zero_editors_btn.clicked.connect(self._zero_editors)
        self.send_all_btn.clicked.connect(self._emit_all)
        self.load_target_btn.clicked.connect(self._load_target_vector)
        self.load_stream_btn.clicked.connect(self._load_stream_vector)

    def _zero_editors(self) -> None:
        for row in self.rows:
            row.set_editor_value(0.0)

    def _emit_all(self) -> None:
        self.send_all_requested.emit(self.editor_vector())

    def _emit_single(self, joint_idx: int) -> None:
        vector = list(self._latest_target)
        vector[joint_idx] = self.rows[joint_idx].editor_value()
        self.send_single_requested.emit(vector)

    def _load_target_vector(self) -> None:
        for idx, value in enumerate(self._latest_target):
            self.rows[idx].set_editor_value(float(value))

    def _load_stream_vector(self) -> None:
        for idx, value in enumerate(self._latest_stream):
            self.rows[idx].set_editor_value(float(value))

    def editor_vector(self) -> List[float]:
        return [row.editor_value() for row in self.rows]

    def set_recording_state(self, recording: bool) -> None:
        self.record_btn.setEnabled(not recording)
        self.stop_record_btn.setEnabled(recording)
        self.record_btn.setText('Recording...' if recording else 'Record')

    def push(self, snapshot) -> None:  # noqa: ANN001
        self._latest_target = [float(v) for v in snapshot.current_target_raw.tolist()]
        backend_state = snapshot.backend_state
        for idx, row in enumerate(self.rows):
            state_pos = None
            torque = None
            if backend_state is not None:
                state_pos = float(backend_state.joint_positions[idx])
                torque = float(backend_state.joint_torques[idx])
                self._latest_stream[idx] = state_pos
            row.update_stream(state_pos, torque)
        self.state_panel.update_snapshot(snapshot)
