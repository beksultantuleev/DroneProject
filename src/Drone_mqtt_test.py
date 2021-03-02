from pyparrot.Minidrone import Mambo
import paho.mqtt.client as mqtt
# import paho.mqtt.client as mqttClient
import time

class Drone:
    def __init__(self, drone_mac):
        self.drone_mac = drone_mac
        self.mambo = Mambo(self.drone_mac, use_wifi=True)
        self.start_measure = False
        self.mambo.set_user_sensor_callback(self.sensor_callback, args=None)

        self.broker_address = "localhost"  # Broker address
        self.port_id = 1883  # Broker port
        self.pos = []
        self.connect_mqtt()

    def connect_mqtt(self):
        mqtt_client = mqtt.Client()
        mqtt_client.connect(self.broker_address, self.port_id)
        mqtt_client.subscribe('Position2')

        self.mqtt_client = mqtt_client
        self.mqtt_client.message_callback_add(
                "Position2", self.on_position)

    def on_position(self, client, userdata, message):
        try:
            data = message.payload.split()
            # data = message.payload[2:-2].split()
            # self.pos = [float(data[0]), float(data[1]), float(data[2])]
            # print(pos)
            print(data)
        except:
            raise
    
    def run(self):
        # print("Starting mqtt background loop")
        self.mqtt_client.loop_start()

    def stop_client(self):
        self.mqtt_client.loop_stop()

    def start_and_prepare(self):
        success = self.mambo.connect(num_retries=3)
        print(f"Connection established >>{success}")

        if (success):
            self.mambo.smart_sleep(1)
            self.mambo.ask_for_state_update()
            print(
                f"Battery level is >> {self.mambo.sensors.__dict__['battery']}%")
            self.mambo.smart_sleep(1)

            print("Taking off!")
            self.mambo.safe_takeoff(3)
    
            if self.mambo.sensors.flying_state != 'emergency':

                print('Sensor calibration...')
                while self.mambo.sensors.speed_ts == 0:
                    continue
                self.start_measure = True

                print('getting first state')
                while self.current_state == []:
                    continue
                '''after this function you need to feed action function such as go to xyz '''
            
    def land_and_disconnect(self):
        print('Landing...')
        self.mambo.safe_land(3)
        self.mambo.smart_sleep(2)
        print('Disconnecting...')
        self.mambo.disconnect()

    def sensor_callback(self, args):
        pass

if __name__ == '__main__':
    ark = Drone("84:20:96:91:73:F1")
    ark.run() #start loop
    while True:
        print(ark.pos)
        print("hello")
        time.sleep(0.5)
    # ark.mambo.safe_takeoff(2)
    # ark.mambo.hover()
    # ark.mambo.smart_sleep(2)
    # ark.land_and_disconnect()
    # ark.stop_client()