"""Dark theme for state machine GUI client."""

STYLE = """
QMainWindow { background-color: #1e1e26; }
QWidget { color: #e0e0e0; font-family: 'Segoe UI', 'Microsoft YaHei UI', sans-serif; font-size: 12px; }
QGroupBox { border: 2px solid #3d3d4d; border-radius: 8px; margin-top: 1ex; font-weight: bold; padding: 12px; color: #b0b0c0; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #00e5ff; }
QPushButton { background-color: #3d3d4d; color: #e0e0e0; border: none; border-radius: 4px; padding: 8px 12px; min-width: 60px; font-weight: 500; }
QPushButton:hover { background-color: #505066; }
QPushButton:pressed { background-color: #606080; }
QPushButton:disabled { background-color: #2a2a35; color: #606070; }
QPushButton#start_btn { background-color: #2e7d32; }
QPushButton#start_btn:hover { background-color: #388e3c; }
QPushButton#stop_btn { background-color: #c62828; }
QPushButton#stop_btn:hover { background-color: #d32f2f; }
QPushButton#connect_btn { background-color: #1565c0; }
QPushButton#connect_btn:hover { background-color: #1976d2; }
QPushButton#policy_btn { background-color: #6a1b9a; }
QPushButton#policy_btn:hover { background-color: #7b1fa2; }
QLineEdit { background-color: #2d2d3a; border: 1px solid #3d3d4d; border-radius: 4px; padding: 6px 8px; color: #e0e0e0; }
QLineEdit:focus { border-color: #00e5ff; }
QDoubleSpinBox { background-color: #2d2d3a; border: 1px solid #3d3d4d; border-radius: 4px; padding: 4px; color: #e0e0e0; }
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { background-color: #3d3d4d; border: none; width: 20px; }
QSpinBox { background-color: #2d2d3a; border: 1px solid #3d3d4d; border-radius: 4px; padding: 4px; color: #e0e0e0; }
QSpinBox::up-button, QSpinBox::down-button { background-color: #3d3d4d; border: none; width: 20px; }
QPlainTextEdit { background-color: #1a1a22; border: 1px solid #3d3d4d; border-radius: 4px; font-family: 'Consolas', monospace; font-size: 11px; color: #b0b0c0; }
QTableWidget { background-color: #1e1e26; border: 1px solid #3d3d4d; gridline-color: #2d2d3a; color: #e0e0e0; }
QTableWidget::item { padding: 4px; }
QHeaderView::section { background-color: #2d2d3a; color: #b0b0c0; padding: 6px; border: none; border-right: 1px solid #3d3d4d; border-bottom: 1px solid #3d3d4d; font-weight: 600; }
QLabel#mode_label { font-size: 16px; font-weight: bold; padding: 8px; color: #00e5ff; }
QLabel#info_label { color: #8080a0; font-size: 10px; font-weight: 600; }
QLabel#val { font-family: 'Consolas', monospace; font-size: 11px; color: #e0e0e0; }
"""
