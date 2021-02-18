from pyparrot.Minidrone import Mambo
import numpy as np


class Drone():
    def __init__(self, drone_mac):
        self.drone_mac = drone_mac
        # self.mambo.set_user_sensor_callback(self.mambo.sensors.sensors_dict, args=None)
    
    def sensor_callback(self, args):
        pass
    
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
        print(f"Battery level is >> {self.mambo.sensors.__dict__['battery']}%")
        self.mambo.smart_sleep(2)

        print("Taking off!")
        self.mambo.safe_takeoff(2)

        self.mambo.set_user_sensor_callback(self.sensor_callback, args=None) #this needs to put after take off. wont show pos if u put before
        
        self.mambo.smart_sleep(2) #it needs to give time for data collection
        self.flight_function(None)

        print("Landing")
        self.mambo.safe_land(3)
        print([self.mambo.sensors.__dict__["sensors_dict"]['DronePosition_posx'], self.mambo.sensors.__dict__["sensors_dict"]['DronePosition_posy'],self.mambo.sensors.__dict__["sensors_dict"]['DronePosition_posz']])

        self.mambo.disconnect()
    
