from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Slot

class SubscribersMethods(QWidget):
    def __init__(self, label):
        super().__init__()
        self._label = label

    @Slot() # we let QT to know that even if this function is called in another thread, still manage it in the QT's main thread anyway
    def update_coordinates(self, x, y):
        self._label.setText(f'{x},{y}')
        return (x,y)
        