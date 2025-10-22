from PySide6.QtGui import Qt, QKeyEvent
from RobotGui.core.control.robot_controller import RobotController

class Keyboard_Command:
    def __init__(self, robot_controller_instance : RobotController):
        print("Entered Keyboard detection init")
        self.robot_control = robot_controller_instance
        self.magnitude = 0         
        self.angle = 0         
        self.pressed_keys = set()
        self.arm_state = 0   # -1 = down, 0 = default, 1 = up
        self.gripper_state = 0  # 0 = open, 1 = closed 

    def handle_key_event(self, key_event: QKeyEvent):
        key = key_event.key()
        self.pressed_keys.add(key)
        # Diagonals
        if {Qt.Key.Key_W, Qt.Key.Key_D}.issubset(self.pressed_keys):
            self.magnitude = 1
            self.angle = 45
        elif {Qt.Key.Key_S, Qt.Key.Key_D}.issubset(self.pressed_keys):
            self.magnitude = 1
            self.angle = 135
        elif {Qt.Key.Key_S, Qt.Key.Key_A}.issubset(self.pressed_keys):
            self.magnitude = 1
            self.angle = 225
        elif {Qt.Key.Key_W, Qt.Key.Key_A}.issubset(self.pressed_keys):
            self.magnitude = 1
            self.angle = 315

        elif key == Qt.Key.Key_W:
            self.magnitude = 1
            self.angle = 0       # forward
        elif key == Qt.Key.Key_D:
            self.magnitude = 1
            self.angle = 90      # right
        elif key == Qt.Key.Key_S:
            self.magnitude = 1
            self.angle = 180     # backward
        elif key == Qt.Key.Key_A:
            self.magnitude = 1
            self.angle = 270     # left
        else:
            self.magnitude = 0

        # Arm control
        if key == Qt.Key.Key_Y:
            self.arm_state = 1    # up
        elif key == Qt.Key.Key_N:
            self.arm_state = -1   # down
        else:
            self.arm_state = 0

        # Gripper control
        if key == Qt.Key.Key_Space:
            self.gripper_state = 1 - self.gripper_state 

        self.robot_control.command_list([self.magnitude, self.angle, self.arm_state, self.gripper_state])

    # def handle_key_release(self, key_event: QKeyEvent):
    #     key = key_event.key()
    #     if key in self.pressed_keys:
    #         self.pressed_keys.remove(key)

    #     # Stop movement when no movement key is pressed
    #     if not any(k in self.pressed_keys for k in [Qt.Key.Key_W, Qt.Key.Key_A, Qt.Key.Key_S, Qt.Key.Key_D]):
    #         self.magnitude = 0
    #         self.publish_movement()

    # def publish_movement(self):
    #     msg = self.mqtt._mqttc_pub.publish("robot/movement", f"{self.magnitude},{self.angle}")
    #     msg.wait_for_publish()
    #     print(f"[MQTT] Movement magnitude={self.magnitude}, angle={self.angle}")

    # def publish_arm(self, direction):
    #     msg = self.mqtt._mqttc_pub.publish("robot/arm", direction)
    #     msg.wait_for_publish()
    #     print(f"[MQTT] Arm moved {direction}")

    # def publish_gripper(self, action):
    #     msg = self.mqtt._mqttc_pub.publish("robot/gripper", action)
    #     msg.wait_for_publish()
    #     print(f"[MQTT] Gripper {action}")