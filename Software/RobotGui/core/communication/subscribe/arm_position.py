from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion
import paho.mqtt.subscribe as subscribe

class Arm_Position():
    def __init__(self, coordinates_slot, address, port):
        self.sub_arm = MC(CallbackAPIVersion.VERSION2)
        self.slot = coordinates_slot
        self.sub_arm.on_connect = self._coordinates_on_connect
        self.sub_arm.connect(address, port)
        self.sub_arm.loop_start()
    
    def _coordinates_on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print('Failed to connect to arm position. Retrying..')
        else:
            subscribe.callback(self.callback, 'robot/arm_position')

    def callback(self, client, userinfo, message):
        theita_str: str = message.payload.decode()
        slot(float(theita_str)) # type: ignore


