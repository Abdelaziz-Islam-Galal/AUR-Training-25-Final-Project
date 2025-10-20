from PySide6.QtWidgets import QWidget, QLabel,QGraphicsView,QGraphicsScene,QGraphicsRectItem,QGraphicsEllipseItem
from PySide6.QtGui import  QResizeEvent, QFont,QColor,Qt,QTransform
from PySide6.QtCore import QTimer

class Minimap(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        font = QFont()
        font.setPointSize(16)

        self._aspect_ratio = 1
        self._square_size=None

        self._scene=QGraphicsScene(self)
        self._view=QGraphicsView(self._scene,self)
        self._view.setRenderHint(self._view.renderHints() | self._view.renderHints().Antialiasing)
        self._view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        #setting background item
        self._background=QGraphicsRectItem()
        self._background.setBrush(QColor(200,200,200))
        self._scene.addItem(self._background)
        #setting robot item
        self._robot=QGraphicsEllipseItem()
        self._robot.setBrush(QColor(255,0,0))
        self._scene.addItem(self._robot)

        
        self._coords_label = QLabel(self)
        self._coords_label.setFont(font)
        
        # self._x = 0
        # self._y = 0

        self._coords_timer = QTimer()
        self._coords_timer.timeout.connect(self.update_coordinates)
        self._coords_timer.setInterval(17)  #around 60 FPS
        self._coords_timer.start()
        

    def resizeEvent(self, event: QResizeEvent) -> None:
        super().resizeEvent(event)
        

        window_size = self.size()

        window_width = window_size.width()
        window_height = window_size.height()

        self._square_size = min(window_width, window_height)


        x_offset = (window_width - self._square_size) // 2
        y_offset = 0

        self._scene.setSceneRect(0, 0, self._square_size, self._square_size)
        transform = QTransform()
        transform.translate(0, self._square_size)
        transform.scale(1, -1)
        self._view.setTransform(transform)

        # update background and robot
        self._background.setRect(0, 0, self._square_size, self._square_size)

        


        self._view.setGeometry(x_offset, y_offset, self._square_size, self._square_size)
        self._coords_label.move(x_offset + 5, y_offset)

    def showEvent(self, event):
        super().showEvent(event)
        self.init_minimap()

    def init_minimap(self):
        #self._view.setGeometry(0, 0, self.width(), self.height())
        window_size = self.size()
        window_width = window_size.width()
        window_height = window_size.height()
        self._square_size = min(window_width, window_height)
        self._scene.setSceneRect(0,0,self._square_size,self._square_size)
        transform=QTransform()
        transform.translate(0,self._square_size)
        transform.scale(1,-1)
        self._view.setTransform(transform)
        self._background.setRect(0, 0, self._square_size, self._square_size)
        self._robot_coords=(0,0)
        self._robot.setRect(self._robot_coords[0],self._robot_coords[1],10,10)

    
    def update_coordinates(self):
        #client subscribe function to be called let's say it's called new_coords
        '''real_coord=new_coords()
        self._coords_label.setText(f'x:{real_coord[0]}, y:{real_coord[1]}')
        real_coord=real_coord*self._square_size/3
        self._robot.setPos(real_coord)'''
            