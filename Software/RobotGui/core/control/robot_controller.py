from .modes import Mode
from RobotGui.core.communication.publish.movement import Movement_Publish
from RobotGui.core.communication.client import Mqtt

class RobotController():
    def __init__(self):
        self.mode=Mode.Manual
        self._mqtt = Mqtt(None)
        self._move = Movement_Publish(self._mqtt)
    
    def setMode(self,new_mode:Mode):
        print(f'switching to {new_mode.name}')
        self.mode=new_mode
    def choose_mode(self,cmd:list):  #receives command from GUI window 
        if self.mode==Mode.Manual:
            self.manual_control(cmd)
        elif self.mode==Mode.Full_Auto:
            self.full_auto()
    def manual_control(self,cmd):
        #self._move.publish_body_movement(cmd)
        #self._move.publish_arm_movement(cmd)
        #self._move.publish_gripper_movement(cmd)
        ...
    def full_auto(self):
        ...
