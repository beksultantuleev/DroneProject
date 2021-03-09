import paho.mqtt.client as mqtt
import threading
import time
class MqttSubscriber:
    def __init__(self, brokerip=None, brokerport=1883, topic=None):
        self.__brokerip = brokerip
        self.__brokerport = brokerport
        self.__topic = topic
        self.__client = mqtt.Client()
        self.__client.on_connect = self.__on_connect
        self.__client.on_disconnect = self.__on_disconnect
        self.__client.on_message = self.__on_message
        self.message = None
        self.receive = True
        self.start_loc = None
        self.end_loc = None
        self.pos = [0, 0, 0]

    def __on_connect(self, client, userdata, flags, rc):
        print("** subscriber connection **")
        self.__client.subscribe(self.__topic, qos=0)

    def __on_disconnect(self, client, userdata, rc):
        print("** disconnection **")

    def __on_message(self, client, userdata, message):
        # print(message.topic)
        data = message.payload[2:-2].split()
        self.pos = [float(data[0]), float(data[1]), float(data[2])]


    def start(self):
        thread = threading.Thread(target=self.__subscribe)
        thread.start()

    def __subscribe(self):
        self.__client.connect(self.__brokerip, self.__brokerport)
        self.__client.loop_forever()
        # self.__client.loop_start()

    def stop(self):
        self.__client.unsubscribe(self.__topic)
        self.__client.disconnect()


if __name__ == '__main__':
    mqttSubscriber = MqttSubscriber("localhost", topic="Position2")
    mqttSubscriber.start()
    while(1):
        print(mqttSubscriber.pos)
        time.sleep(0.5)
        print("hello beck")
    # mqttSubscriber.get_pos()