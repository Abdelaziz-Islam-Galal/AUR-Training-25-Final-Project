from RobotGui.core.communication.client import Mqtt

class Movement_Publish():
    def __init__(self, mqtt_client):
        print("entered MovementPublish init")
        self.mqtt = mqtt_client # the mqtt instance -> the one that setted publishing
        self.flag = 0 # if zero send coordinates else send angle of direction
        self.x, self.y = 0, 0 # coordinates
        self.theta = 0 # angle of direction
        self.direction = 0 # arm (-1 = down, 0 = nothing, 1 = up)
        self.state = 0 # gripper state

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
        if self.flag == 0:
            Mqtt.publish_msg(self.mqtt, "/movement/body", self.flag, self.x, self.y)
            print(f"Published body movement: {self.flag},{self.x},{self.y}")
        elif self.flag == 1:
            Mqtt.publish_msg(self.mqtt, "/movement/body", self.flag, self.theta)
            print(f"Published body movement: {self.flag},{self.theta}")


    def publish_arm_movement(self):
        Mqtt.publish_msg(self.mqtt, "/movement/arm", self.theta)
        print(f"Published arm movement: {self.direction}")
    
    def publish_gripper_state(self):
        Mqtt.publish_msg(self.mqtt, "/movement/gripper", self.state)
        print(f"Published gripper state: {self.state}")

