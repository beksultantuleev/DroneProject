import paho.mqtt.client as mqtt
import re
import numpy as np
import numpy.matlib
import array as arr
import json
import paho.mqtt.client as mqttClient
import time
import collections
import atexit

class DroneUWB:
    def __init__(self):
        self.broker_address = "192.168.1.200"  # Broker address
        self.port_id = 1883  # Broker port
        self.subscriptions_qos = [("Position2", 0)]
        self.pos = []
    def mqtt_connection(self):
        client = mqtt.Client()
        # client.on_log = on_log
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.connect(self.broker_address, self.port_id)
        client.loop_start()
        self.get_pos()
    
    def on_connect(self, client, userdata, flags, rc):
        # print("Successfully connected to MQTT with result code %s" % str(rc))
        # print("before message_callback_add 1")
        client.message_callback_add("Position2", self.pos_callback)
        # print("after message_callback_add")

        (result, _) = client.subscribe(self.subscriptions_qos)
        # if (result == mqtt.MQTT_ERR_SUCCESS):
            # print("Successfully subscribed to MQTT topics with result code %s" %
                # str(result))

    def on_message(self, client, userdata, message):
        print("Received: Topic: %s Body: %s", message.topic, message.payload)

    def pos_callback(self, client, userdata, message):
        try:
            data = message.payload[2:-2].split()
            self.pos = [float(data[0]), float(data[1]), float(data[2])]
            # print(pos)
        except:
            raise
    
    def get_pos(self):
        return self.pos

if __name__ == '__main__':
    mambo1 = DroneUWB()
    while(True):
       
        mambo1.mqtt_connection()
        mambo1.pos