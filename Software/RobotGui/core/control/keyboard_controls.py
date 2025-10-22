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

    def handle_key_press(self, key_event: QKeyEvent):
        key = key_event.key()
        self.pressed_keys.add(key)
        self._update_movement()
        self.robot_control.command_list([self.magnitude, self.angle, self.arm_state, self.gripper_state])

    def handle_key_release(self, key_event: QKeyEvent):
        key = key_event.key()
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
        self._update_movement()
        self.robot_control.command_list([self.magnitude, self.angle, self.arm_state, self.gripper_state])

    def _update_movement(self):
        # Reset movement
        self.magnitude = 0
        self.angle = 0
        
        # Check for movement keys
        has_w = Qt.Key.Key_W in self.pressed_keys
        has_a = Qt.Key.Key_A in self.pressed_keys
        has_s = Qt.Key.Key_S in self.pressed_keys
        has_d = Qt.Key.Key_D in self.pressed_keys
        
        # Diagonal movements (priority)
        if has_w and has_d:
            self.magnitude = 1
            self.angle = 45    # Forward-right
        elif has_s and has_d:
            self.magnitude = 1
            self.angle = 135   # Backward-right
        elif has_s and has_a:
            self.magnitude = 1
            self.angle = 225   # Backward-left
        elif has_w and has_a:
            self.magnitude = 1
            self.angle = 315   # Forward-left
        
        # Cardinal directions (only if not already in diagonal)
        elif has_w and self.magnitude == 0:
            self.magnitude = 1
            self.angle = 0     # Forward
        elif has_d and self.magnitude == 0:
            self.magnitude = 1
            self.angle = 90    # Right
        elif has_s and self.magnitude == 0:
            self.magnitude = 1
            self.angle = 180   # Backward
        elif has_a and self.magnitude == 0:
            self.magnitude = 1
            self.angle = 270   # Left

    def _update_arm_and_gripper(self, key_event: QKeyEvent):
        key = key_event.key()
        
        # Arm control (only on key press)
        if key == Qt.Key.Key_Y:
            self.arm_state = 1    # up
        elif key == Qt.Key.Key_N:
            self.arm_state = -1   # down
        
        # Gripper control (toggle on key press)
        if key == Qt.Key.Key_Space:
            self.gripper_state = 1 - self.gripper_state

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