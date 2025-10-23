from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout
from PySide6.QtCore import Slot, QTimer
from PySide6.QtGui import QFont
from RobotGui.core.cv.cv import Camera


class InfoDisplay(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        font = QFont()
        font.setPointSize(10)

        self._qr_label = QLabel()
        self._qr_label.setStyleSheet('background-color: #1E90FF;')
        self._qr_label.setFont(font)

        self._arm_label = QLabel()
        self._arm_label.setStyleSheet('background-color: #3CB371;')
        self._arm_label.setFont(font)

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0,0,0,0)
        self._layout.addWidget(self._arm_label)
        self._layout.addWidget(self._qr_label)

        # use a single Camera instance and read its last_qr property (no parentheses)
        self._camera = Camera()
        self.update_QR(self._camera.last_qr)

        # poll for updates so the UI reflects new QR scans
        self._qr_timer = QTimer(self)
        self._qr_timer.setInterval(200)
        self._qr_timer.timeout.connect(lambda: self.update_QR(self._camera.last_qr))
        self._qr_timer.start()

    @Slot(str)
    def update_QR(self, coord: str):
        self._qr_label.setText(coord)
