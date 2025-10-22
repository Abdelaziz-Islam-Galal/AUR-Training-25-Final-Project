from PySide6.QtWidgets import QWidget, QLabel
from PySide6.QtGui import QImage, QPixmap, QResizeEvent
from PySide6.QtCore import QTimer, Slot
from RobotGui.core.cv.cv import Camera


class CameraDisplay(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self._aspect_ratio = 4/3
        
        self._camera_device = Camera()

        self._frame_view = QLabel(self)
        self._frame_view.setScaledContents(True)

        self.update_view()

        self._camera_timer = QTimer()
        self._camera_timer.timeout.connect(self.update_view)
        self._camera_timer.setInterval(17)  #around 60 FPS
        self._camera_timer.start()

    def resizeEvent(self, event: QResizeEvent) -> None:
        super().resizeEvent(event)

        window_size = self.size()

        window_width = window_size.width()
        window_height = window_size.height()

        if window_width > window_height*self._aspect_ratio:
            new_width = int(window_height*self._aspect_ratio)
            new_height = window_height
        else:
            new_width = window_width
            new_height = int(window_width/self._aspect_ratio)

        x_offset = (window_width - new_width)//2
        y_offset = (window_height - new_height)//2

        self._frame_view.setGeometry(x_offset, y_offset, new_width, new_height)

    @Slot()
    def update_view(self):
        frame = self._camera_device.frame
        if frame is not None:
            image = QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format.Format_BGR888)
            self._frame_view.setPixmap(QPixmap.fromImage(image))