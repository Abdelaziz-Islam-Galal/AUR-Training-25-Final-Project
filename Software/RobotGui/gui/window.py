from PySide6.QtWidgets import QWidget, QMainWindow, QHBoxLayout, QVBoxLayout, QSizePolicy, QStatusBar, QPushButton
from PySide6.QtGui import QResizeEvent, QKeyEvent, Qt
from PySide6.QtCore import QTimer, Slot

from RobotGui.gui.camera_display import CameraDisplay
from RobotGui.gui.minimap import Minimap
from RobotGui.gui.settings import Settings
from RobotGui.gui.info_display import InfoDisplay

from RobotGui.core.communication.client import Mqtt
from RobotGui.core.communication.publish.movement import Movement_Publish
from RobotGui.core.communication.subscribe.Subscribers_methods import SubscribersMethods

import RobotGui.gui.settings as settings

class Window(QMainWindow):
    def __init__(self):

        super().__init__()

        global _mqtt
        _mqtt = Mqtt()

        global subscriber
        subscriber = SubscribersMethods()
        global movement_publisher
        movement_publisher = Movement_Publish(_mqtt)

        self.setWindowTitle('Robot GUI')

        self.setCentralWidget(CentralWidget())
        self.centralWidget().setMinimumSize(770, 420)
        self._aspect_ratio = 11/6

        self.setStatusBar(StatusBar())

        self.centralWidget()._other_widgets.start_button.clicked.connect(self._start_timer)
        

        # to make the window focus on keyboard presses
        self.setFocusPolicy(Qt.StrongFocus)
        self.show()

    @Slot()
    def _start_timer(self):
        self.statusBar()._countdown_flag = True

    def keyPressEvent(self, event: QKeyEvent):
        if settings.keyboard_instructions is not None:
            settings.keyboard_instructions.handle_key_press(event)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent):
        if settings.keyboard_instructions is not None:
            settings.keyboard_instructions.handle_key_release(event)
        super().keyReleaseEvent(event)

    def resizeEvent(self, event: QResizeEvent) -> None:
        super().resizeEvent(event)

        window_size = self.size()
        
        window_width = window_size.width()
        window_height = window_size.height() - self.statusBar().height()

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

        self._minimap_widget = Minimap(subcriber_data=subscriber)
        self._minimap_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._v_layout.addWidget(self._minimap_widget, stretch=1)

        self._info_widget = InfoDisplay()
        self._info_widget.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self._v_layout.addWidget(self._info_widget, stretch=0)

        self._settings_widget = Settings(_mqtt)
        self._settings_widget.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self._v_layout.addWidget(self._settings_widget, stretch=0)

        self.start_button = QPushButton('Start Timer')
        self.start_button.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self._v_layout.addWidget(self.start_button, stretch=0)

        self._mqtt_sub_coordinates = _mqtt.setup_coordinates(self._minimap_widget.subscriber_data.update_coordinates)


class StatusBar(QStatusBar):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self._robot_status = False
        self._seconds = 600
        self._countdown_flag = False

        self._timer = QTimer()
        self._timer.timeout.connect(self._update_status)
        self._timer.setInterval(1000)
        self._timer.start()

    @Slot()
    def _update_status(self):
        if self._countdown_flag and self._seconds>0:
            self._seconds -=1

        self._robot_status = ...
        if self._robot_status:
            self.showMessage(f'{self._seconds//60}:{(self._seconds%60):02}   |   Robot Connected') 
        else:
            self.showMessage(f'{self._seconds//60}:{(self._seconds%60):02}   |   Robot Disonnected') 