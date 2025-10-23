from PySide6.QtWidgets import QWidget, QComboBox
from PySide6.QtCore import Slot
from PySide6.QtGui import QFont
from RobotGui.core.control.modes import Mode
from RobotGui.core.control.robot_controller import RobotController
from RobotGui.core.control.controller import Controller
from RobotGui.core.control.keyboard_controls import Keyboard_Command

keyboard_instructions = None

class Settings(QComboBox):
    def __init__(self, mqtt_client, parent: QWidget | None = None):
        super().__init__(parent)

        font = QFont()
        font.setPointSize(10)
        self.setFont(font)
        self.setPlaceholderText('Choose Mode')
        self.addItems(['Manual', 'Semi-Auto', 'Full-Auto'])
        self.currentIndexChanged.connect(self._on_mode_change)
        self._robot_controller = RobotController(mqtt_client)
        
        try:
            self._controller_logic = Controller(self._robot_controller)
        except Exception as e:
            print(f"failed to connect to controller: {e}")

        global keyboard_instructions
        keyboard_instructions = Keyboard_Command(self._robot_controller)



    @Slot()
    def _on_mode_change(self, index):
        if(index==0):
            self._robot_controller.setMode(Mode.Manual)
        elif(index==1):
            self._robot_controller.setMode(Mode.Semi_Auto)
        elif(index==2):
            self._robot_controller.setMode(Mode.Full_Auto)