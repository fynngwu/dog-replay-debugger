from __future__ import annotations

from PySide6.QtWidgets import QTableWidget, QTableWidgetItem, QHeaderView


class JointTable(QTableWidget):
    HEADERS = ['Joint', 'Target', 'MuJoCo', 'Robot', 'MuJoCo Err', 'Robot Err']

    def __init__(self):
        super().__init__(12, len(self.HEADERS))
        self.setHorizontalHeaderLabels(self.HEADERS)
        header = self.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        self.verticalHeader().setVisible(False)
        self.setAlternatingRowColors(True)
        self.setSelectionBehavior(QTableWidget.SelectRows)

    def update_rows(self, rows):
        for r, row in enumerate(rows):
            for c, value in enumerate(row):
                text = value if isinstance(value, str) else ('-' if value is None else f'{value:.4f}')
                item = self.item(r, c)
                if item is None:
                    item = QTableWidgetItem(text)
                    self.setItem(r, c, item)
                else:
                    item.setText(text)
