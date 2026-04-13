from __future__ import annotations

from PySide6.QtWidgets import (
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHeaderView,
    QLabel,
    QPlainTextEdit,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from replay_core.constants import JOINT_NAMES


class BackendStatePanel(QWidget):
    def __init__(self, title: str = 'Backend Stream State', parent: QWidget | None = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        summary_box = QGroupBox(title)
        summary_layout = QGridLayout(summary_box)
        summary_layout.setContentsMargins(10, 10, 10, 10)
        summary_layout.setSpacing(8)

        self.value_labels: dict[str, QLabel] = {}
        fields = [
            ('OK', 'ok'),
            ('Enabled', 'enabled'),
            ('Worker', 'worker_started'),
            ('Busy', 'busy'),
            ('Init', 'init_in_progress'),
            ('Queue', 'queue_size'),
            ('KP', 'kp'),
            ('KD', 'kd'),
            ('Age', 'age'),
        ]
        for i, (text, key) in enumerate(fields):
            row = i // 3
            col = (i % 3) * 2
            summary_layout.addWidget(QLabel(f'{text}:'), row, col)
            value = QLabel('--')
            value.setStyleSheet("font-family: 'Consolas';")
            summary_layout.addWidget(value, row, col + 1)
            self.value_labels[key] = value
        layout.addWidget(summary_box)

        self.table = QTableWidget(len(JOINT_NAMES), 6)
        self.table.setHorizontalHeaderLabels(['Joint', 'StatePos', 'Torque', 'ActiveTarget', 'LastSent', 'UI Target'])
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for row, name in enumerate(JOINT_NAMES):
            self.table.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, 6):
                self.table.setItem(row, col, QTableWidgetItem('--'))
        layout.addWidget(self.table)

        error_box = QGroupBox('last_error')
        error_layout = QFormLayout(error_box)
        error_layout.setContentsMargins(10, 10, 10, 10)
        self.last_error_label = QLabel('--')
        self.last_error_label.setWordWrap(True)
        self.last_error_label.setStyleSheet("font-family: 'Consolas';")
        error_layout.addRow(self.last_error_label)
        layout.addWidget(error_box)

        raw_box = QGroupBox('Raw JSON')
        raw_layout = QVBoxLayout(raw_box)
        raw_layout.setContentsMargins(10, 10, 10, 10)
        self.raw_text = QPlainTextEdit()
        self.raw_text.setReadOnly(True)
        self.raw_text.setMinimumHeight(140)
        raw_layout.addWidget(self.raw_text)
        layout.addWidget(raw_box)

    def update_snapshot(self, snapshot) -> None:  # noqa: ANN001
        backend_state = None if snapshot is None else snapshot.backend_state
        age = None if snapshot is None else snapshot.backend_state_age_s
        if backend_state is None:
            for label in self.value_labels.values():
                label.setText('--')
            self.last_error_label.setText('--')
            self.raw_text.setPlainText('')
            for row in range(len(JOINT_NAMES)):
                for col in range(1, 6):
                    self.table.item(row, col).setText('--')
            return

        def fmt(value) -> str:
            if value is None:
                return '--'
            if isinstance(value, bool):
                return 'true' if value else 'false'
            if isinstance(value, int):
                return str(value)
            if isinstance(value, float):
                return f'{value:.4f}'
            return str(value)

        self.value_labels['ok'].setText(fmt(backend_state.ok))
        self.value_labels['enabled'].setText(fmt(backend_state.enabled))
        self.value_labels['worker_started'].setText(fmt(backend_state.worker_started))
        self.value_labels['busy'].setText(fmt(backend_state.busy))
        self.value_labels['init_in_progress'].setText(fmt(backend_state.init_in_progress))
        self.value_labels['queue_size'].setText(fmt(backend_state.queue_size))
        self.value_labels['kp'].setText(fmt(backend_state.kp))
        self.value_labels['kd'].setText(fmt(backend_state.kd))
        self.value_labels['age'].setText('--' if age is None else f'{age:.3f}s')
        self.last_error_label.setText(backend_state.last_error or '(empty)')
        self.raw_text.setPlainText(backend_state.raw_json)

        for row in range(len(JOINT_NAMES)):
            values = [
                float(backend_state.joint_positions[row]),
                float(backend_state.joint_torques[row]),
                float(backend_state.target_joint_positions[row]),
                float(backend_state.last_sent_joint_positions[row]),
                float(snapshot.current_target[row]),
            ]
            for col, value in enumerate(values, start=1):
                self.table.item(row, col).setText(f'{value:.4f}')
