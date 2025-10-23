from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion
import paho.mqtt.subscribe as subscribe

class Gripper_State():
    def __init__(self, coordinates_slot, address, port):
        self.sub_arm = MC(CallbackAPIVersion.VERSION2)
        self.slot = coordinates_slot
        self.sub_arm.on_connect = self._coordinates_on_connect
        self.sub_arm.connect(address, port)
        self.sub_arm.loop_start()
    
    def _coordinates_on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print('Failed to connect to gripper state. Retrying..')
        else:
            subscribe.callback(self.callback, 'gripper_state')

    def callback(self, client, userinfo, message):
        state_str: str = message.payload.decode()
        self.slot(int(state_str)) # type: ignore


