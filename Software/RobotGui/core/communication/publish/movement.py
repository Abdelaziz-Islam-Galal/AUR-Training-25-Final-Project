from RobotGui.core.communication.client import Mqtt

class Movement_Publish():
    def __init__(self, mqtt_client):
        print("entered MovementPublish init")
        self.mqtt = mqtt_client # the mqtt instance -> the one that setted publishing
        self.x, self.y = 0, 0
        self.theta = 0

    # def handle_key_event(self, key_event):
    #     key = key_event.key()
        
    #     if key == Qt.Key.Key_Left:
    #         self.x -= 1
    #     elif key == Qt.Key.Key_Right:
    #         self.x += 1
    #     elif key == Qt.Key.Key_Up:
    #         self.y += 1
    #     elif key == Qt.Key.Key_Down:
    #         self.y -= 1
    #     else:
    #         return

    #     self.publish_movement()

    def publish_body_movement(self):
        Mqtt.publish_msg(self.mqtt, "robot/body/movement", self.x, self.y)
        print(f"Published body movement: {self.x}, {self.y}")

    def publish_arm_movement(self):
        Mqtt.publish_msg(self.mqtt, "robot/arm/movement", self.theta)
        print(f"Published arm movement: {self.theta}")
    

