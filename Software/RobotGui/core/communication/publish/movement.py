import time

class Movement_Publish():
    def __init__(self, mqtt_client):
        print("entered MovementPublish init")
        self.mqtt = mqtt_client # the mqtt instance -> the one that setted publishing
        self.x, self.y = 0, 0
        self.theita = 0

    # def handle_key_event(self, key_event):
    #     key = key_event.key()
        
    #     if key == Qt.Key.Key_Left:
    #         self.x -= 1
    #     elif key == Qt.Key.Key_Right:
    #         self.x += 1
    #     elif key == Qt.Key.Key_Up:
    #         self.y += 1
    #     elif key == Qt.Key.Key_Down:
    #         self.y -= 1
    #     else:
    #         return

    #     self.publish_movement()

    def publish_body_movement(self):
        msg = self.mqtt._mqttc_pub.publish("robot/body/movement", f'{self.x},{self.y}')
        if self.mqtt.unacked_publish is not None:
            self.mqtt.unacked_publish.add(msg.mid)
        while len( self.mqtt.unacked_publish):
            time.sleep(0.1)
        msg.wait_for_publish()
        print(f"Published body movement: {self.x}, {self.y}")

    def publish_arm_movement(self):
        msg = self.mqtt.pub.publish("robot/arm/movement", f'{self.theita}')
        if self.mqtt.unacked_publish is not None:
            self.mqtt.unacked_publish.add(msg.mid)
        while len(self.mqtt.unacked_publish):
            time.sleep(0.1)
        msg.wait_for_publish()
        print(f"Published arm movement: {self.theita}")
    

