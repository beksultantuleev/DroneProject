from numpy import matrixlib
from pyparrot.Minidrone import Mambo
# from subscriber import MqttSubscriber

class Drone:
    def __init__(self, drone_mac, use_wifi):
        self.drone_mac = drone_mac
        self.use_wifi = use_wifi
        self.mambo = Mambo(self.drone_mac, use_wifi=use_wifi)
        self.start_measure = False
        self.mambo.set_user_sensor_callback(self.sensor_callback, args=None)
        # self.mqttSubscriber = MqttSubscriber()


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
            self.mambo.safe_takeoff(5) #we have extended from 3 to 10 
    
            if self.mambo.sensors.flying_state != 'emergency':

                print('Sensor calibration...')
                while self.mambo.sensors.speed_ts == 0:
                    continue
                self.start_measure = True

                # print('getting first state')
                # while self.current_state == []:
                #     continue
                # '''after this function you need to feed action function such as go to xyz '''
            
    def land_and_disconnect(self):
        print('Landing...')
        self.mambo.safe_land(3)
        self.mambo.smart_sleep(1)
        print('Disconnecting...')
        self.mambo.disconnect()
        # self.mqttSubscriber.stop()



    def sensor_callback(self, args):
        pass


if __name__ == "__main__":
    mac = "D0:3A:49:F7:E6:22"
    mambo1 = Drone(mac, False)
    mambo1.start_and_prepare()
    # mambo1.mambo.safe_takeoff(5)
    mambo1.mambo.fly_direct(0,30,0,1,1)
    # mambo1.mambo.safe_takeoff(5)
    # mambo1.start_and_prepare()
    # # mambo1.mambo.turn_degrees(90)
    mambo1.land_and_disconnect()

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
    # "84:20:96:6c:22:67" <<<uwb attached new drone