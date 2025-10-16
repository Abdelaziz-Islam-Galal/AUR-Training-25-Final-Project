from PySide6.QtWidgets import QWidget, QMainWindow, QHBoxLayout, QVBoxLayout, QSizePolicy
from PySide6.QtGui import QResizeEvent

from RobotGui.gui.camera_display import CameraDisplay
from RobotGui.gui.minimap import Minimap
from RobotGui.gui.settings import Settings


class Window(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('Robot GUI')

        self.setCentralWidget(CentralWidget())
        self.centralWidget().setMinimumSize(550, 300)
        self._aspect_ratio = 11/6

        self.show()

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

        self.centralWidget().setGeometry(x_offset, y_offset, new_width, new_height)
    

class CentralWidget(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self._h_layout = QHBoxLayout(self)
        self._h_layout.setContentsMargins(0,0,0,0)
        self._h_layout.setSpacing(0)
        
        self._camera_widget = CameraDisplay()
        self._h_layout.addWidget(self._camera_widget, stretch=8)

        self._other_widgets = WidgetGrid()
        self._h_layout.addWidget(self._other_widgets, stretch=3)


class WidgetGrid(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self._v_layout = QVBoxLayout(self)

        self._minimap_widget = Minimap()
        self._minimap_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._v_layout.addWidget(self._minimap_widget, stretch=1)

        self._settings_widget = Settings()
        self._settings_widget.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self._v_layout.addWidget(self._settings_widget, stretch=0)