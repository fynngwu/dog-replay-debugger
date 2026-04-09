from __future__ import annotations

from collections import deque

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
    QHeaderView,
)

from replay_core.constants import JOINT_NAMES

KNEE_INDICES = [8, 9, 10, 11]
KNEE_NAMES = [JOINT_NAMES[i] for i in KNEE_INDICES]

try:
    import pyqtgraph as pg
except Exception:  # pragma: no cover
    pg = None


class KneeDebugPanel(QWidget):
    target_changed = Signal(list, float)
    record_requested = Signal()
    stop_record_requested = Signal()

    def __init__(self):
        super().__init__()
        self._updating_widgets = False
        self._build_ui()
        self._init_buffers()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        top_row = QHBoxLayout()
        top_row.setSpacing(12)
        top_row.addWidget(self._build_control_box(), stretch=2)
        top_row.addWidget(self._build_record_box(), stretch=1)
        layout.addLayout(top_row)

        layout.addWidget(self._build_table_box())
        layout.addWidget(self._build_plot_box(), stretch=1)

    def _build_control_box(self) -> QWidget:
        box = QGroupBox('Knee parameter debug')
        grid = QGridLayout(box)
        grid.setContentsMargins(12, 12, 12, 12)
        grid.setSpacing(8)

        desc = QLabel('Each knee has its own slider. Replay display/compare uses daemon-equivalent clamped targets.')
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #b0b0c0;")
        grid.addWidget(desc, 0, 0, 1, 5)

        self.knee_checks: dict[int, QCheckBox] = {}
        self.knee_sliders: dict[int, QSlider] = {}
        self.knee_spins: dict[int, QDoubleSpinBox] = {}
        self.knee_labels: dict[int, QLabel] = {}

        for col, (joint_idx, joint_name) in enumerate(zip(KNEE_INDICES, KNEE_NAMES)):
            cb = QCheckBox(joint_name)
            cb.setChecked(True)
            cb.stateChanged.connect(lambda _state, idx=joint_idx: self._on_check_changed(idx))
            self.knee_checks[joint_idx] = cb
            grid.addWidget(cb, 1, col)

        self.zero_btn = QPushButton('Zero all')
        self.zero_btn.clicked.connect(self._zero_all)
        grid.addWidget(self.zero_btn, 1, 4)

        for col, (joint_idx, joint_name) in enumerate(zip(KNEE_INDICES, KNEE_NAMES)):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(-3140, 3140)
            slider.setValue(0)
            slider.valueChanged.connect(lambda _val, idx=joint_idx: self._slider_changed(idx))
            self.knee_sliders[joint_idx] = slider
            grid.addWidget(slider, 2, col)

            spin = QDoubleSpinBox()
            spin.setRange(-3.14, 3.14)
            spin.setDecimals(4)
            spin.setSingleStep(0.01)
            spin.valueChanged.connect(lambda _val, idx=joint_idx: self._spin_changed(idx))
            self.knee_spins[joint_idx] = spin
            grid.addWidget(spin, 3, col)

            lbl = QLabel('raw=0.0000  clamped=0.0000')
            lbl.setStyleSheet("color: #b0b0c0; font-family: 'Consolas'; font-size: 10px;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.knee_labels[joint_idx] = lbl
            grid.addWidget(lbl, 4, col)

        return box

    def _build_record_box(self) -> QWidget:
        box = QGroupBox('Recording')
        layout = QVBoxLayout(box)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        text = QLabel('This duplicates the global record control, so knee tuning and replay capture can be done in one tab.')
        text.setWordWrap(True)
        text.setStyleSheet("color: #b0b0c0;")
        layout.addWidget(text)

        btn_row = QHBoxLayout()
        self.record_btn = QPushButton('Record')
        self.stop_record_btn = QPushButton('Stop Record')
        self.stop_record_btn.setEnabled(False)
        self.record_btn.clicked.connect(self.record_requested)
        self.stop_record_btn.clicked.connect(self.stop_record_requested)
        btn_row.addWidget(self.record_btn)
        btn_row.addWidget(self.stop_record_btn)
        layout.addLayout(btn_row)
        layout.addStretch(1)
        return box

    def _build_table_box(self) -> QWidget:
        box = QGroupBox('Knee values')
        layout = QVBoxLayout(box)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        self.table = QTableWidget(len(KNEE_INDICES), 7)
        self.table.setHorizontalHeaderLabels(['Joint', 'RawReq', 'Target', 'MuJoCo', 'Robot', 'Err_M', 'Err_R'])
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for row, name in enumerate(KNEE_NAMES):
            self.table.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, 7):
                self.table.setItem(row, col, QTableWidgetItem('--'))
        layout.addWidget(self.table)
        return box

    def _build_plot_box(self) -> QWidget:
        box = QGroupBox('Knee curves')
        layout = QVBoxLayout(box)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        if pg is None:
            layout.addWidget(QLabel('pyqtgraph is not installed'))
            self.plot = None
            self.target_curves = {}
            self.real_curves = {}
            return box

        self.plot = pg.PlotWidget()
        self.plot.setBackground('#1e1e26')
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend()
        layout.addWidget(self.plot)

        colors = {
            8: '#00e5ff',
            9: '#69f0ae',
            10: '#ffd740',
            11: '#ff8a65',
        }
        self.target_curves = {}
        self.real_curves = {}
        for idx in KNEE_INDICES:
            self.target_curves[idx] = self.plot.plot(name=f'{JOINT_NAMES[idx]} target', pen=pg.mkPen(colors[idx], width=2))
            self.real_curves[idx] = self.plot.plot(name=f'{JOINT_NAMES[idx]} real', pen=pg.mkPen(colors[idx], width=2, style=Qt.PenStyle.DashLine))
        return box

    def _init_buffers(self) -> None:
        self._x = deque(maxlen=400)
        self._counter = 0
        self._target_hist = {idx: deque(maxlen=400) for idx in KNEE_INDICES}
        self._real_hist = {idx: deque(maxlen=400) for idx in KNEE_INDICES}

    def selected_knees(self) -> list[int]:
        return [idx for idx, cb in self.knee_checks.items() if cb.isChecked()]

    def set_recording_state(self, recording: bool) -> None:
        self.record_btn.setEnabled(not recording)
        self.stop_record_btn.setEnabled(recording)
        self.record_btn.setText('Recording...' if recording else 'Record')

    def set_slider_feedback(self, knee_idx: int, raw_value: float, clamped_value: float) -> None:
        if knee_idx in self.knee_labels:
            self.knee_labels[knee_idx].setText(f'raw={raw_value:.4f}  clamped={clamped_value:.4f}')

    def _slider_changed(self, joint_idx: int) -> None:
        if self._updating_widgets:
            return
        self._updating_widgets = True
        try:
            self.knee_spins[joint_idx].setValue(float(self.knee_sliders[joint_idx].value()) / 1000.0)
        finally:
            self._updating_widgets = False
        self._emit_live_target(joint_idx)

    def _spin_changed(self, joint_idx: int) -> None:
        if self._updating_widgets:
            return
        self._updating_widgets = True
        try:
            self.knee_sliders[joint_idx].setValue(int(round(float(self.knee_spins[joint_idx].value()) * 1000.0)))
        finally:
            self._updating_widgets = False
        self._emit_live_target(joint_idx)

    def _on_check_changed(self, _joint_idx: int) -> None:
        pass

    def _zero_all(self) -> None:
        for idx in KNEE_INDICES:
            self.knee_spins[idx].setValue(0.0)

    def _emit_live_target(self, joint_idx: int) -> None:
        self.target_changed.emit([joint_idx], float(self.knee_spins[joint_idx].value()))

    def push(self, snapshot) -> None:  # noqa: ANN001
        real_src = snapshot.robot_state if snapshot.robot_state is not None else snapshot.mujoco_state
        self._counter += 1
        self._x.append(self._counter)
        for idx in KNEE_INDICES:
            self._target_hist[idx].append(float(snapshot.current_target[idx]))
            self._real_hist[idx].append(None if real_src is None else float(real_src.positions[idx]))

        self._update_table(snapshot)
        self._update_plot()

    def _update_table(self, snapshot) -> None:  # noqa: ANN001
        mj_pos = None if snapshot.mujoco_state is None else snapshot.mujoco_state.positions
        rb_pos = None if snapshot.robot_state is None else snapshot.robot_state.positions
        mj_err = snapshot.mujoco_error
        rb_err = snapshot.robot_error
        for row, idx in enumerate(KNEE_INDICES):
            values = [
                f'{float(snapshot.current_target_raw[idx]):.4f}',
                f'{float(snapshot.current_target[idx]):.4f}',
                '--' if mj_pos is None else f'{float(mj_pos[idx]):.4f}',
                '--' if rb_pos is None else f'{float(rb_pos[idx]):.4f}',
                '--' if mj_err is None else f'{float(mj_err[idx]):.4f}',
                '--' if rb_err is None else f'{float(rb_err[idx]):.4f}',
            ]
            for col, value in enumerate(values, start=1):
                self.table.item(row, col).setText(value)

    def _update_plot(self) -> None:
        if pg is None or self.plot is None:
            return
        x = list(self._x)
        for idx in KNEE_INDICES:
            visible = self.knee_checks[idx].isChecked()
            target_curve = self.target_curves[idx]
            real_curve = self.real_curves[idx]
            if not visible:
                target_curve.setData([], [])
                real_curve.setData([], [])
                continue
            target_curve.setData(x, list(self._target_hist[idx]))
            real_curve.setData(x, [0.0 if v is None else v for v in self._real_hist[idx]])
