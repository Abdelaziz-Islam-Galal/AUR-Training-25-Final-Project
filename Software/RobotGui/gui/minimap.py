##
## This is a placeholder file
##
from PySide6.QtWidgets import QWidget, QLabel
from PySide6.QtGui import QImage, QPixmap, QResizeEvent, QFont
import cv2


class Minimap(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        font = QFont()
        font.setPointSize(16)

        self._aspect_ratio = 1

        self._frame_view = QLabel(self)
        self._frame_view.setScaledContents(True)

        self._frame = cv2.imread('minimap_placeholder.png')
        
        frame = self._frame
        if frame is not None:
            image = QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format.Format_BGR888)
            self._frame_view.setPixmap(QPixmap.fromImage(image))

        self._coords_label = QLabel(self)
        self._coords_label.setFont(font)
        self._x = 0
        self._y = 0
        self._coords_label.setText(f'x:{self._x}, y:{self._y}')

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
        y_offset = 0

        self._frame_view.setGeometry(x_offset, y_offset, new_width, new_height)
        self._coords_label.move(x_offset + 5, y_offset)