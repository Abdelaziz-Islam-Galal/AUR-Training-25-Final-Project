from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion

from RobotGui.core.communication.subscribe.coordinates import Coordinates

import time
from threading import Thread

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

    def publish_msg(self, mqtt_client, topic, *args):
        if len(args) == 1:
            message = str(args[0])
        else:
            message = ",".join(str(arg) for arg in args)
        
        self._pub_thread = Thread(target=self._publishing, args=[mqtt_client, topic, message], daemon=True)
        self._pub_thread.start()
        
    def _publishing(self, mqtt_client, topic, message):    
        msg = mqtt_client.pub.publish(topic, message)
        if mqtt_client.unacked_publish is not None:
            mqtt_client.unacked_publish.add(msg.mid)
        while len(mqtt_client.unacked_publish):
            time.sleep(0.1)
        msg.wait_for_publish()

