from PySide6.QtWidgets import QWidget, QMainWindow, QHBoxLayout, QVBoxLayout, QSizePolicy
from PySide6.QtGui import QResizeEvent
from PySide6.QtCore import QTimer
from RobotGui.gui.camera_display import CameraDisplay
from RobotGui.gui.minimap import Minimap
from RobotGui.gui.settings import Settings

from PySide6.QtGui import QKeyEvent, Qt
from RobotGui.core.communication.client import Mqtt
from RobotGui.core.communication.subscribe.Subscribers_methods import SubscribersMethods
from RobotGui.core.communication.publish.movement import Movement_Publish

# _mqtt = None

class Window(QMainWindow):
    def __init__(self):
        super().__init__()

        global _mqtt
        _mqtt = Mqtt()

        self.setWindowTitle('Robot GUI')

        self.setCentralWidget(CentralWidget())
        self.centralWidget().setMinimumSize(550, 300)
        self._aspect_ratio = 11/6
        self._coords_timer=QTimer()
        self._camera_timer = QTimer()
        # self._camera_timer.timeout.connect(Minimap.update_coordinates)
        self._camera_timer.setInterval(17)  #around 60 FPS
        self._camera_timer.start()

        #self._subscribers = SubscribersMethods(self._coords_label)
        
        self._movement_publisher = Movement_Publish(_mqtt)

        # to make the window focus on keyboard presses
        self.setFocusPolicy(Qt.StrongFocus)
        self.show()

    key = None
    def keyPressEvent(self, event: QKeyEvent) -> None:
        self._movement_publisher.handle_key_event(event)
        super().keyPressEvent(event)

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

        self._mqtt_sub_coordinates = _mqtt.setup_coordinates(self._minimap_widget._subscriber.update_coordinates)
