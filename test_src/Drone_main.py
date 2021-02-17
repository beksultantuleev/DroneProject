from pyparrot.Minidrone import Mambo
import numpy as np


class Drone():
    def __init__(self, drone_mac):
        self.drone_mac = drone_mac
        # self.mambo.set_user_sensor_callback(self.sensor_cb, args=None)
    
    def flight_function(self, args):
        ''' any flight function in other classes '''
        pass

    def run(self):
        self.mambo = Mambo(self.drone_mac.islower(), use_wifi=True)
        success = self.mambo.connect(num_retries=3)
        print(f"Connection established >> {success}")
        print("Sleeping")
        self.mambo.smart_sleep(2)
        self.mambo.ask_for_state_update()
        print(f"Battery level is >> {self.mambo.sensors.__dict__['battery']}")
        self.mambo.smart_sleep(2)

        print("Taking off!")
        self.mambo.safe_takeoff(3)

        self.flight_function(None)

        print("Landing")
        self.mambo.safe_land(3)

        self.mambo.disconnect()
    
