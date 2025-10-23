from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion
import paho.mqtt.subscribe as subscribe

class Coordinates():
    def __init__(self, coordinates_slot, address, port):
        self.sub_coordinates = MC(CallbackAPIVersion.VERSION2)
        self.slot = coordinates_slot
        self.sub_coordinates.on_connect = self._coordinates_on_connect
        self.sub_coordinates.connect(address, port)
        self.sub_coordinates.loop_start()
    
    def _coordinates_on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print('Failed to connect to coordinates. Retrying..')
        else:
            self.sub_coordinates.subscribe('robot_coordinates')
            self.sub_coordinates.on_message = self.callback

    def callback(self, client, userinfo, message):
        payload_str: str = message.payload.decode()
        coords = payload_str.split(',')
        if len(coords) >= 2:
            try:
                self.slot(float(coords[0]), float(coords[1]), float(coords[2]))
            except (ValueError, IndexError) as e:
                print(f"Error parsing coordinates: {e}")
