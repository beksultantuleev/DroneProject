from pyparrot.Minidrone import Mambo
import numpy as np


class Drone():
    def __init__(self, test_flying, drone_mac, use_wifi=True):
        self.test_flying = test_flying
        self.drone_mac = drone_mac
        self.use_wifi = use_wifi
        self.mambo = Mambo(self.drone_mac.islower(), use_wifi=self.use_wifi)
        

    def execute(self):
        print("Connecting...")
        success = self.mambo.connect(num_retries=3)
        print(f"Connection established >> {success}")
        if success:
            self.mambo.smart_sleep(1)
            self.mambo.ask_for_state_update()
            self.mambo.smart_sleep(2)

            self.mambo.safe_takeoff(3)

            self.mambo.set_user_sensor_callback(self.sensor_cb, args=None)

            self.flight_func(None)

            self.mambo.disconnect()
    
    def flight_func(self, args):
        '''try to use :
                while self.mambo.sensors.speed_ts == 0:
                    continue'''
        pass

    def sensor_cb(self, args):
        pass
