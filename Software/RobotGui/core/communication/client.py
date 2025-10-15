from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion

from RobotGui.core.communication.subscribe.coordinates import Coordinates
from RobotGui.core.communication.publish.movement import Movement_Publish

class Mqtt():
    def __init__(self, coordinates_slot, address = 'localhost', port = 1883):
        self.publishing_setup() #for all publishing to all topics

        coordinates_setup = Coordinates(coordinates_slot, address, port)
        
    def publishing_setup(self):
        self.unacked_publish = set()
        self.pub = MC(CallbackAPIVersion.VERSION2)
        self.pub.user_data_set(self.unacked_publish)
        self.pub.connect("localhost", 1883)
        self.pub.loop_start()

