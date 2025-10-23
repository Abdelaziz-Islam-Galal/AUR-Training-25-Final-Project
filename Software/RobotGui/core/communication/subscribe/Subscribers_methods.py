from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Slot

class SubscribersMethods(QWidget):
    def __init__(self):
        super().__init__()

        self.xValue = 0
        self.yValue = 0
        self.thetaValue = 0

    @Slot(float,float) # we let QT to know that even if this function is called in another thread, still manage it in the QT's main thread anyway
    def update_coordinates(self, x, y, theta):
        self.xValue = x
        self.yValue = y
        self.thetaValue = theta
    
    @property
    def coordinates(self):
        return (self.xValue, self.yValue, self.thetaValue)
    
    # @Slot() # we let QT to know that even if this function is called in another thread, still manage it in the QT's main thread anyway
    # def arm_position(self, theta):
    #      theta
    
    # @Slot() # we let QT to know that even if this function is called in another thread, still manage it in the QT's main thread anyway
    # def gripper_state(self, state):
    #     self._label.setText(f'{state}')
    #     return state
        