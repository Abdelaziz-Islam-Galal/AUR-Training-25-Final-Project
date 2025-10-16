##
## This is a placeholder file
##
from PySide6.QtWidgets import QWidget, QPushButton, QSizePolicy, QVBoxLayout

class Settings(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self._layout = QVBoxLayout(self)
        self._button = QPushButton('Placeholder')
        self._layout.addWidget(self._button)