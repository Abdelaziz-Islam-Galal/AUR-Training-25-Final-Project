from .modes import Mode

class RobotController():
    def __init__(self):
        self.mode=Mode.Manual
    
    def setMode(self,new_mode:Mode):
        print(f'switching to {new_mode.name}')
        self.mode=new_mode
    def choose_mode(self,cmd):  #receives command from GUI window 
        if self.mode==Mode.Manual:
            self.manual_control(cmd)
        elif self.mode==Mode.Full_Auto:
            self.full_auto()
    def manual_control(self,cmd):
        ... #sends it to the communication function 
    def full_auto(self):
        ...
