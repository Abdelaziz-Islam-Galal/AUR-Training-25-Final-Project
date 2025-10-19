from PySide6.QtWidgets import QWidget, QLabel, QSizePolicy
from PySide6.QtCore import Slot, QTimer
from RobotGui.core.cv.cv import Camera


class QRDisplay(QLabel):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.setStyleSheet("background-color: red;")

        # use a single Camera instance and read its last_qr property (no parentheses)
        self._camera = Camera()
        self.update_QR(self._camera.last_qr)

        # poll for updates so the UI reflects new QR scans
        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(lambda: self.update_QR(self._camera.last_qr))
        self._timer.start()

    @Slot(str)
    def update_QR(self, coord: str):
        self.setText(coord)
