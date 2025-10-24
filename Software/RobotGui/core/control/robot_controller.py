from .modes import Mode
from RobotGui.core.communication.publish.movement import Movement_Publish
from RobotGui.core.communication.client import Mqtt
from RobotGui.core.auto.delivery import delivery
from RobotGui.core.auto.road_to_box import road_to_box
from RobotGui.core.cv.color_detection_and_recognition import FormMask
from RobotGui.core.auto.aotounmous import qr_auto
from time import time,sleep
import cv2
from threading import Thread

class RobotController():
    def __init__(self, mqtt_client):
        self.mode=Mode.Manual
        self._move = Movement_Publish(mqtt_client)
    
    def setMode(self,new_mode:Mode):
        print(f'switching to {new_mode.name}')
        self.mode=new_mode
        if self.mode==Mode.Full_Auto:
            self.full_auto()
    def command_list(self,cmd:list):  #receives command from GUI window 
        if self.mode==Mode.Manual:
            self.manual_control(cmd)
        
    def manual_control(self,cmd):
        self._move.publish_body_movement(cmd[0], cmd[1])
        self._move.publish_arm_movement(cmd[2])
        self._move.publish_gripper_state(cmd[3])
        
        #self._move.publish_body_movement(cmd)
        #self._move.publish_arm_movement(cmd)
        #self._move.publish_gripper_movement(cmd)
        
    def full_auto(self,):
        from RobotGui.gui.camera_display import camera_device
        from RobotGui.gui.window import movement_publisher

        start=time()
        while time - start < 4*60:
            road_to_box('red')
            delivery('red')
        while True:
            color=self.detect_first_color(['blue','green'])
            if color=='green':
                road_to_box(color)
                delivery(color)
            elif color=='blue':
                qr_thread = Thread(target=qr_auto, daemon=True)
                box_thread = Thread(target=road_to_box, args=(color,), daemon=True)

                qr_thread.start()
                box_thread.start()

                qr_thread.join()
                box_thread.join()
                if not camera_device.timeout:
                    result=camera_device.last_qr
                    movement_publisher.flag=1
                    movement_publisher.x,movement_publisher.y=result
                    movement_publisher.publish_body_movement(0,0) 


    def detect_first_color(self, color_list):
        from RobotGui.gui.camera_display import camera_device
        from RobotGui.gui.window import subscriber,movement_publisher

        while True:
            frame = camera_device.frame
            if frame is None:
                sleep(0.1)
                continue

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            angle_cmd=90
            while True: 
                for color in color_list:
                    mask = FormMask(hsv, color)
                    count = cv2.countNonZero(mask)
                    if count > 5000:  
                        return color
                    _,_,angle=subscriber.coordinates
                if angle > 270:
                    angle_cmd = 270
                elif angle < 90:
                    angle_cmd = 90
                movement_publisher.publish_body_movement(1,angle_cmd)
