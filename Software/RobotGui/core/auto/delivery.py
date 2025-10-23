from RobotGui.core.communication.subscribe.Subscribers_methods import SubscribersMethods
from RobotGui.core.communication.publish.movement import Movement_Publish
from PySide6.QtWidgets import QLabel
from RobotGui.core.cv.cv import Camera
from RobotGui.core.cv.color_detection_and_recognition import ColorDetection,COLORS_BGR,FormMask
from time import sleep
camera=Camera()
publisher=Movement_Publish()
label=QLabel()
subscriber=SubscribersMethods(label)
_,_,angle=subscriber.coordinates
while angle < 90 or angle > 95:
    if angle < 90:
        publisher.publish_body_movement(1,90)
        sleep(0.5)
        publisher.publish_body_movement(0,0)
        
    elif angle > 95:
        publisher.publish_body_movement(1,270)
        sleep(0.5)
        publisher.publish_body_movement(0,0)
    _,_,angle=subscriber.coordinates

#start scanning zone
camera.start_color_thread()
angle_cmd=90
while not camera.detected:
    
    _,_,angle=subscriber.coordinates
    if angle > 270:
        angle_cmd = 270
    elif angle < 90:
        angle_cmd = 90
    publisher.publish_body_movement(1,angle_cmd)
    sleep(0.5)
    publisher.publish_body_movement(0,0)
while not camera.detector.inzone:
    publisher.publish_body_movement(1,0)
    sleep(0.5)
    publisher.publish_body_movement(0,0)
camera.stop_color_thread()
publisher.publish_body_movement(1,0)
sleep(1)
publisher.publish_body_movement(0,0)
publisher.publish_arm_movement(-1)
sleep(0.02)
publisher.publish_gripper_state(0)
_,_,angle=subscriber.coordinates
angle_cmd=90
while angle > 5 or angle < -5:
    if angle > 5:
        angle_cmd=270
        publisher.publish_body_movement(1,angle_cmd)
        sleep(0.5)
        publisher.publish_body_movement(0,0)
    elif angle < -5:
        angle_cmd=90
        publisher.publish_body_movement(1,angle_cmd)
        sleep(0.5)
        publisher.publish_body_movement(0,0)
