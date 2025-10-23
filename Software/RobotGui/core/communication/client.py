from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion

from RobotGui.core.communication.subscribe.coordinates import Coordinates
from RobotGui.core.communication.subscribe.arm_position import Arm_Position
from RobotGui.core.communication.subscribe.gripper_state import Gripper_State

import time
from threading import Thread

class Mqtt():
    def __init__(self, address = 'localhost', port = 1883):
        print("Entered Mqtt")
        self.address = address
        self.port = port

        self.publishing_setup() #for all publishing to all topics

    # setup all topics that will be subscribed to
    def setup_coordinates(self, coordinates_slot):
        self.coordinates = Coordinates(coordinates_slot, self.address, self.port)
        
    def setup_arm_position(self, arm_slot):
        self.arm_position = Arm_Position(arm_slot, self.address, self.port) #-> adds arm_slot as input if this is uncommented

    def setup_gripper_state(self, gripper_slot):
        self.coordinates = Gripper_State(gripper_slot, self.address, self.port)


    # publishing methods: 
    def publishing_setup(self):
        self.unacked_publish = set()
        self.pub = MC(CallbackAPIVersion.VERSION2)
        self.pub.user_data_set(self.unacked_publish)
        self.pub.connect("localhost", 1883)
        self.pub.loop_start()

    # I am calling self with the word mqtt_client to remember this important fact
    def publish_msg(mqtt_client, topic, *args):
        if len(args) == 1:
            message = str(args[0])
        else:
            message = ",".join(str(arg) for arg in args)
        
        mqtt_client._pub_thread = Thread(target=mqtt_client._publishing, args=[topic, message], daemon=True)
        mqtt_client._pub_thread.start()
        
    def _publishing(mqtt_client, topic, message):    
        msg = mqtt_client.pub.publish(topic, message)
        if mqtt_client.unacked_publish is not None:
            mqtt_client.unacked_publish.add(msg.mid)
        while len(mqtt_client.unacked_publish):
            time.sleep(0.1)
        msg.wait_for_publish()

