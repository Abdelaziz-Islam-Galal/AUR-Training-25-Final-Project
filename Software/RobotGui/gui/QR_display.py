from PySide6.QtWidgets import QWidget, QLabel, QSizePolicy, QHBoxLayout
from PySide6.QtCore import Slot, QTimer
from RobotGui.core.cv.cv import Camera

class QRDisplay(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self._label = QLabel(self)
        policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self._label.setSizePolicy(policy)

        layout = QHBoxLayout(self)
        layout.addWidget(self._label)

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
        self._label.setText(coord)
