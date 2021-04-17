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



class UWBconnection:
    def __init__(self):
        self.Connected = False  # global variable for the state of the connection
        self.DEBUG = False
        self.num_anch = 7
        self.num_tag = 5
        self.buffer_size = 100
        self.slot = np.zeros((self.num_tag, self.buffer_size, self.num_anch),
                             dtype=int)  # creo 100 slot toa
        self.tmp = []
        self.DBdata = {}
        self.DBdata['measurement'] = 'TOA_Ranging'
        self.DBdata['tags'] = {}
        self.DBdata['tags'] = {'room': 'lab', 'anchors': 0}
        self.DBdata['fields'] = {}
        self.DBdata['tags']['anchors'] = 6
        self.dist = {'A1': 0, 'A2': 0, 'A3': 0, 'A4': 0, 'A5': 0, 'A6': 0}
        self.dict_keys = list(self.dist.keys())
        self.broker_address = "192.168.1.200"  # Broker address or localhost
        self.port_id = 1883  # Broker port
        self.subscriptions_qos = [("Position2", 0)]


    def on_connect(self, client, userdata, flags, rc):
        print("Successfully connected to MQTT with result code %s" % str(rc))
        print("before message_callback_add 1")
        client.message_callback_add("Position2", self.pos_callback)
        print("after message_callback_add")

        (result, _) = client.subscribe(self.subscriptions_qos)
        if (result == mqtt.MQTT_ERR_SUCCESS):
            print("Successfully subscribed to MQTT topics with result code %s" %
                str(result))


    def on_message(self, client, userdata, message):
        print("Received: Topic: %s Body: %s", message.topic, message.payload)


    def pos_callback(self, client, userdata, message):
        try:
            data = message.payload.split()
            print(data)
        except:
            raise


    def main(self):
        # logger = logging.getLogger('root')
        # logging.basicConfig(format='[%(asctime)s %(levelname)s: %(funcName)20s] %(message)s', level=logging.DEBUG)

        client = mqtt.Client()
        # client.on_log = on_log
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.connect(self.broker_address, self.port_id)
        client.loop_forever()




if __name__ == '__main__':
    test = UWBconnection()
    test.main()
