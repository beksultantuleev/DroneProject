"""
    Drone.py
    Marcus Abate | 16.30
    12/1/18

    This class encapsulates many of the standard methods and objects used to
    run tests and scripts on the Parrot Mambo drone with vision.
"""

from pyparrot.Minidrone import Mambo
# from pyparrot.DroneVisionGUI import DroneVisionGUI

class Drone:
    def __init__(self, mambo_addr):
        self.mamboAddr = mambo_addr
        self.mambo = Mambo(self.mamboAddr, use_wifi=True)
        self.mambo.set_user_sensor_callback(self.sensor_callback, args=None)


    def run(self):
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
            
            self.flight_function(None)

            self.mambo.smart_sleep(2)
            self.mambo.disconnect()

    def flight_function(self, args):
        pass


    def sensor_callback(self, args):
        pass
