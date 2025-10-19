from PySide6.QtWidgets import QWidget, QComboBox
from PySide6.QtCore import Slot
from RobotGui.core.control.modes import Mode
from RobotGui.core.control.robot_controller import RobotController


class Settings(QComboBox):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self.setPlaceholderText('Choose Mode')
        self.addItems(['Manual', 'Semi-Auto', 'Full-Auto'])
        self.currentIndexChanged.connect(self._on_mode_change)
        self._robot_controller = RobotController()


    @Slot()
    def _on_mode_change(self, index):
        if(index==0):
            self._robot_controller.setMode(Mode.Manual)
        elif(index==1):
            self._robot_controller.setMode(Mode.Semi_Auto)
        elif(index==2):
            self._robot_controller.setMode(Mode.Full_Auto)