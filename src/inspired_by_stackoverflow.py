from future import unicode_literals
from pyparrot.Minidrone import Mambo
import paho.mqtt.client as mqtt
import logging
from interpolate import Interpolation
from datetime import datetime, timedelta

# logger = logging.getLogger('robotgroup')

class DroneGroup:
    def init(self):
        self.mqtt_client = None
        # self.config = config
        self.connect_mqtt()
        self.broker_address = "localhost"
        self.port_id = 1883
        self.mambos = {} # robot names
        # self._last_seen = {}  # datetime or None


    def connect_mqtt(self):
        # mqtt_config = self.config['mqtt-server']
        #
        client = mqtt.Client()
        # if 'username' in mqtt_config:
        #     mqtt_client.username_pw_set(mqtt_config['username'],
        #                                 mqtt_config['password'])
        client.connect(self.broker_address, self.port_id)
        # logger.info("MQTT connecting to '%s'", mqtt_config['host'])
        client.subscribe('Position2')
        # client.subscribe('/robot/+/$name$')

        self.mqtt_client = client
        self.mqtt_client.message_callback_add(
                "Position2", self._on_position)
        # self.mqtt_client.message_callback_add(
        #         "/robot/+/$name$", self._on_name)

    def _on_position(self, client, userdata, msg):
        try:
            # logging.debug('Got alive message: %s: %s',
            #                     msg.topic, msg.payload)
            # parts = msg.topic.split('/')
            data = msg.payload.split()
            print(data)
            # robot_id = parts[2]
            # logging.info("Robot #%s state is %i", robot_id, int(msg.payload))
            # if int(msg.payload) == 0:
            #     self._last_seen[robot_id] = None
            # else:
            #     self._last_seen[robot_id] = datetime.now()
        except:
            raise ValueError



    # def _on_name(self, client, userdata, msg):
    #     logging.debug('Got name message: %s: %s',
    #                         msg.topic, msg.payload)
    #     parts = msg.topic.split('/')
    #     robot_id = parts[2]
    #     logging.info("Robot #%s name is %s", robot_id, repr(msg.payload))
    #     try:
    #         self.mambos[robot_id] = msg.payload.decode('us-ascii')
    #     except UnicodeDecodeError:
    #         self.mambos[robot_id] = 'WFT'


    
    def run(self):
        print("Starting mqtt background loop")
        # logger.info("Starting mqtt background loop")
        self.mqtt_client.loop_start()
        print("mqtt loop started")
        # logger.info("mqtt loop started")


    # def mambo_list(self):
    #     for id in sorted(self._last_seen):
    #         yield self.get_mambo(id=id)

    # def get_mambo(self, id, online=None):
    #     if online is None:
    #         online = self._last_seen[id]
    #     return Drone(drone_mac=drone_mac,
    #                  mqtt_client=self.mqtt_client,
    #                  name=self.mambos.get(id, None),
    #                  )

# inspired by http://stackoverflow.com/a/17115473/7554925

class Drone:
    def init(self, drone_mac, mqtt_client):
        self.drone_mac = drone_mac
        self.mambo = Mambo(self.drone_mac, use_wifi=True)
        self.start_measure = False
        self.mambo.set_user_sensor_callback(self.sensor_callback, args=None)
        self.mqtt_client = mqtt_client

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
