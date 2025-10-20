from PySide6.QtGui import Qt, QKeyEvent

class MovementPublish:
    def __init__(self, mqtt_client):
        print("Entered MovementPublish init")
        self.mqtt = mqtt_client
        self.flag = 0         
        self.angle = 0         
        self.pressed_keys = set()
        self.arm_state = 0   # -1 = down, 0 = default, 1 = up
        self.gripper_state = 0  # 0 = open, 1 = closed 

    def handle_key_event(self, key_event: QKeyEvent):
        key = key_event.key()
        self.pressed_keys.add(key)
        # Diagonals
        if {Qt.Key.Key_W, Qt.Key.Key_D}.issubset(self.pressed_keys):
            self.flag = 1
            self.angle = 45
        elif {Qt.Key.Key_S, Qt.Key.Key_D}.issubset(self.pressed_keys):
            self.flag = 1
            self.angle = 135
        elif {Qt.Key.Key_S, Qt.Key.Key_A}.issubset(self.pressed_keys):
            self.flag = 1
            self.angle = 225
        elif {Qt.Key.Key_W, Qt.Key.Key_A}.issubset(self.pressed_keys):
            self.flag = 1
            self.angle = 315

        elif key == Qt.Key.Key_W:
            self.flag = 1
            self.angle = 0       # forward
        elif key == Qt.Key.Key_D:
            self.flag = 1
            self.angle = 90      # right
        elif key == Qt.Key.Key_S:
            self.flag = 1
            self.angle = 180     # backward
        elif key == Qt.Key.Key_A:
            self.flag = 1
            self.angle = 270     # left

        # Arm control
        elif key == Qt.Key.Key_Y:
            self.arm_state = 1    # up
            self.publish_arm()
        elif key == Qt.Key.Key_N:
            self.arm_state = -1   # down
            self.publish_arm()

        # Gripper control
        elif key == Qt.Key.Key_Space:
            self.gripper_state = 1 - self.gripper_state 
            self.publish_gripper()

        self.publish_movement()

    def handle_key_release(self, key_event: QKeyEvent):
        key = key_event.key()
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)

        # Stop movement when no movement key is pressed
        if not any(k in self.pressed_keys for k in [Qt.Key.Key_W, Qt.Key.Key_A, Qt.Key.Key_S, Qt.Key.Key_D]):
            self.flag = 0
            self.publish_movement()

    def publish_movement(self):
        msg = self.mqtt._mqttc_pub.publish("robot/movement", f"{self.flag},{self.angle}")
        msg.wait_for_publish()
        print(f"[MQTT] Movement flag={self.flag}, angle={self.angle}")

    def publish_arm(self, direction):
        msg = self.mqtt._mqttc_pub.publish("robot/arm", direction)
        msg.wait_for_publish()
        print(f"[MQTT] Arm moved {direction}")

    def publish_gripper(self, action):
        msg = self.mqtt._mqttc_pub.publish("robot/gripper", action)
        msg.wait_for_publish()
        print(f"[MQTT] Gripper {action}")