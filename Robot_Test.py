from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion
from time import sleep
from random import random
import paho.mqtt.subscribe as subscribe

_mqttc = MC(CallbackAPIVersion.VERSION2)
x = 0
y = 0


slot = None
def callback(client, userinfo, message):
    global x, y
    payload_str: str = message.payload.decode()
    coords = payload_str.split(',')
    x = float(coords[0])
    y = float(coords[1])
    print(f"Updated coordinates: x={x}, y={y}")
    slot(x, y) # type: ignore

def update_test(x: float, y: float):
        print(f'Robot is at: {x}, {y}')

def setup(test_slot, address = 'localhost', port = 1883):
    global slot
    slot = test_slot
    _mqttc.on_connect = _on_connect
    _mqttc.on_message = callback
    _mqttc.connect(address, port)
    _mqttc.loop_start()


def _on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print('Failed to connect. Retrying..')
    else:
        _mqttc.subscribe('/movement/test')


setup(update_test)

print("Robot started; waiting for movement commads in  via topic: robot/movement")

while True:
    sleep(1)
    x = random() * 10
    y = random() * 10
    _mqttc.publish('robot_coordinates', f'{x:.2f},{y:.2f}')
    print(f"Published coordinates: {x},{y}")