"""Modern dark theme styles for Dog Replay Debugger."""

MODERN_STYLE = """
/* Main window background */
QMainWindow {
    background-color: #1e1e26;
}

/* Base widget styles */
QWidget {
    color: #e0e0e0;
    font-family: 'Segoe UI', 'Microsoft YaHei UI', sans-serif;
    font-size: 12px;
}

/* Group box with modern border */
QGroupBox {
    border: 2px solid #3d3d4d;
    border-radius: 8px;
    margin-top: 1ex;
    font-weight: bold;
    padding: 12px;
    color: #b0b0c0;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
    color: #00e5ff;
}

/* Push buttons */
QPushButton {
    background-color: #3d3d4d;
    color: #e0e0e0;
    border: none;
    border-radius: 4px;
    padding: 8px 12px;
    min-width: 60px;
    font-weight: 500;
}

QPushButton:hover {
    background-color: #505066;
}

QPushButton:pressed {
    background-color: #606080;
}

QPushButton:disabled {
    background-color: #2a2a35;
    color: #606070;
}

/* Special button colors */
QPushButton#start_btn {
    background-color: #2e7d32;
}
QPushButton#start_btn:hover {
    background-color: #388e3c;
}

QPushButton#stop_btn {
    background-color: #c62828;
}
QPushButton#stop_btn:hover {
    background-color: #d32f2f;
}

QPushButton#connect_btn {
    background-color: #1565c0;
}
QPushButton#connect_btn:hover {
    background-color: #1976d2;
}

/* Line edits */
QLineEdit {
    background-color: #2d2d3a;
    border: 1px solid #3d3d4d;
    border-radius: 4px;
    padding: 6px 8px;
    color: #e0e0e0;
    selection-background-color: #505066;
}

QLineEdit:focus {
    border: 1px solid #00e5ff;
}

/* Spin boxes */
QSpinBox {
    background-color: #2d2d3a;
    border: 1px solid #3d3d4d;
    border-radius: 4px;
    padding: 4px;
    color: #e0e0e0;
}

QSpinBox::up-button, QSpinBox::down-button {
    background-color: #3d3d4d;
    border: none;
    width: 20px;
}

QSpinBox::up-button:hover, QSpinBox::down-button:hover {
    background-color: #505066;
}

/* Sliders */
QSlider::groove:horizontal {
    border: none;
    height: 6px;
    background-color: #3d3d4d;
    border-radius: 3px;
    margin: 2px 0;
}

QSlider::handle:horizontal {
    background-color: #00e5ff;
    border: none;
    height: 16px;
    width: 16px;
    border-radius: 8px;
    margin: -5px 0;
}

QSlider::handle:horizontal:hover {
    background-color: #00b8cc;
}

QSlider::sub-page:horizontal {
    background-color: #3d3d4d;
    border-radius: 3px;
}

/* Combo boxes */
QComboBox {
    background-color: #2d2d3a;
    border: 1px solid #3d3d4d;
    border-radius: 4px;
    padding: 4px 8px;
    color: #e0e0e0;
}

QComboBox::drop-down {
    border: none;
    background-color: #3d3d4d;
    border-radius: 4px;
    width: 20px;
}

QComboBox QAbstractItemView {
    background-color: #2d2d3a;
    border: 1px solid #3d3d4d;
    selection-background-color: #505066;
    color: #e0e0e0;
}

/* Plain text edits (logs) */
QPlainTextEdit {
    background-color: #1a1a22;
    border: 1px solid #3d3d4d;
    border-radius: 4px;
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 11px;
    color: #b0b0c0;
}

QPlainTextEdit:focus {
    border: 1px solid #00e5ff;
}

/* Scroll bars */
QScrollBar:vertical {
    background-color: #1e1e26;
    width: 12px;
    border-radius: 6px;
    margin: 0;
}

QScrollBar::handle:vertical {
    background-color: #3d3d4d;
    border-radius: 6px;
    min-height: 30px;
}

QScrollBar::handle:vertical:hover {
    background-color: #505066;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}

QScrollBar:horizontal {
    background-color: #1e1e26;
    height: 12px;
    border-radius: 6px;
}

QScrollBar::handle:horizontal {
    background-color: #3d3d4d;
    border-radius: 6px;
    min-width: 30px;
}

QScrollBar::handle:horizontal:hover {
    background-color: #505066;
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0;
}

/* Splitter */
QSplitter::handle {
    background-color: #3d3d4d;
}

QSplitter::handle:hover {
    background-color: #505066;
}

/* Table widget */
QTableWidget {
    background-color: #1e1e26;
    border: 1px solid #3d3d4d;
    gridline-color: #2d2d3a;
    color: #e0e0e0;
}

QTableWidget::item {
    padding: 6px;
}

QTableWidget::item:selected {
    background-color: #505066;
    color: #ffffff;
}

QTableWidget::item:hover {
    background-color: #3d3d4d;
}

QHeaderView::section {
    background-color: #2d2d3a;
    color: #b0b0c0;
    padding: 6px;
    border: none;
    border-right: 1px solid #3d3d4d;
    border-bottom: 1px solid #3d3d4d;
    font-weight: 600;
}

QTableWidget::item:alternate {
    background-color: #252530;
}

/* Status bar style */
.QLabel#status_bar {
    color: #b0b0c0;
    padding: 6px;
    background-color: #2d2d3a;
    border-radius: 4px;
}

/* Status indicator lights */
.QLabel#status_indicator {
    font-family: 'Consolas', monospace;
    font-size: 11px;
    padding: 4px 8px;
    border-radius: 3px;
    background-color: #2d2d3a;
}

.QLabel#status_ok {
    color: #69f0ae;
}

.QLabel#status_warning {
    color: #ffd740;
}

.QLabel#status_error {
    color: #ff5252;
}

.QLabel#status_info {
    color: #00e5ff;
}
"""

JOINT_CARD_STYLE = """
#JointCard {
    background-color: #2d2d3a;
    border-radius: 6px;
    border: 1px solid #3d3d4d;
}

#JointCard:hover {
    border: 1px solid #505066;
}

#JointCard[selected="true"] {
    border: 2px solid #00e5ff;
    background-color: #323240;
}

QLabel#name {
    color: #00e5ff;
    font-weight: bold;
    font-size: 14px;
    padding: 2px;
}

QLabel#label {
    color: #8080a0;
    font-size: 10px;
    padding: 2px;
}

QLabel#val {
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 13px;
    color: #e0e0e0;
    padding: 2px;
}

QLabel#val_error {
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 12px;
    color: #69f0ae;
    padding: 2px;
}

QLabel#val_error_warn {
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 12px;
    color: #ffd740;
    padding: 2px;
}

QLabel#val_error_high {
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 12px;
    color: #ff5252;
    padding: 2px;
}

/* Leg group labels */
QLabel#leg_group {
    color: #b0b0c0;
    font-weight: 600;
    font-size: 11px;
    padding: 4px;
    background-color: #252530;
    border-radius: 4px;
}
"""

DASHBOARD_STYLE = """
#JointDashboard {
    background-color: #1e1e26;
}

#leg_frame {
    background-color: #252530;
    border-radius: 8px;
    border: 1px solid #3d3d4d;
    padding: 8px;
}

QLabel#dashboard_title {
    color: #b0b0c0;
    font-weight: 600;
    font-size: 13px;
    padding: 6px;
}
"""

STATUS_PANEL_STYLE = """
#status_panel {
    background-color: #2d2d3a;
    border-radius: 6px;
    border: 1px solid #3d3d4d;
    padding: 8px;
}

QLabel#status_label {
    color: #8080a0;
    font-size: 10px;
    font-weight: 600;
}

QLabel#status_value {
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 11px;
    color: #e0e0e0;
}

QLabel#status_value_ok {
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 11px;
    color: #69f0ae;
}

QLabel#status_value_warn {
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 11px;
    color: #ffd740;
}

QLabel#status_value_error {
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 11px;
    color: #ff5252;
}
"""
