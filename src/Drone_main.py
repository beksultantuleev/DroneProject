from pyparrot.Minidrone import Mambo

class Drone:
    def __init__(self, drone_mac):
        self.drone_mac = drone_mac
        self.mambo = Mambo(self.drone_mac, use_wifi=True)
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
