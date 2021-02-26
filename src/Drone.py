from pyparrot.Minidrone import Mambo

class Drone:
    def __init__(self, drone_mac):
        self.drone_mac = drone_mac
        self.mambo = Mambo(self.drone_mac, use_wifi=True)
        self.start_measure = False
        self.mambo.set_user_sensor_callback(self.sensor_callback, args=None)


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
