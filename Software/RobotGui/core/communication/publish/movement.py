from PySide6.QtGui import QKeyEvent, Qt
from RobotGui.core.communication.client import Mqtt
import random
import logging

logger = logging.getLogger(__name__)

class Movement_Publish():
    def __init__(self, mqtt_client):
        print("entered MovementPublish init")
        self.mqtt = mqtt_client # the mqtt instance -> the one that setted publishing
        self.direction = 0 # tmp for testing

    # temp for testing
    def handle_key_event(self, key_event):
        ...
    #     key = key_event.key()
        
    #     if key == Qt.Key.Key_Left:
    #         self.direction -= 1
    #     elif key == Qt.Key.Key_Right:
    #         self.direction += 1
    #     # elif key == Qt.Key.Key_Up:
    #     #     self.direction += 1
    #     # elif key == Qt.Key.Key_Down:
    #     #     self.direction -= 1
    #     else:
    #         return
    #     # print(f"Movement detected: magnitude={1}, direction={self.direction}")
    #     self.publish_body_movement([1, self.direction])

    def publish_body_movement(self, magnitude, angle):
        self.flag = 0 # magnitude & angle or coordinates?
        self.x = 0
        self.y = 0

        self.magnitude = magnitude
        self.angle = angle
        if self.flag == 0:
            Mqtt.publish_msg(self.mqtt, "movement_body", self.magnitude, self.angle)
            print(f"Published body movement: {self.magnitude},{self.angle}")
        elif self.flag == 1:
            Mqtt.publish_msg(self.mqtt, "movement_body", self.x, self.y) 
            print(f"Published body movement: {self.x},{self.y}")

    def publish_arm_movement(self, direction):
        self.direction = direction # 3 states -> -1/0/1
        Mqtt.publish_msg(self.mqtt, "movement_arm", self.direction)
        print(f"Published arm movement: {self.direction}")

    def publish_gripper_state(self, open):
        self.open = open # boolean -> 0/1
        Mqtt.publish_msg(self.mqtt, "movement_gripper", self.open)
        print(f"Published gripper state: {self.open}")
