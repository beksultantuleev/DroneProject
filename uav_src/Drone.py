from pyparrot.Minidrone import Mambo
from PositionController import MamboPositionController
from KalmanFilter import KalmanFilterUWB
import numpy as np
from subscriber import MqttSubscriber

class Drone:
    def __init__(self, drone_mac):
        self.drone_mac = drone_mac
        self.mambo = Mambo(self.drone_mac, use_wifi=True)
        self.start_measure = False
        self.mambo.set_user_sensor_callback(self.sensor_callback, args=None)
        self.mqtt = MqttSubscriber()
        self.controller = MamboPositionController()
        # self.kalmanfilter = MamboKalman([0, 0, 0], [0, 0, 0])
        self.q = np.ones((3, 1))
        self.p = p = np.zeros((3, 3))
        # self.kalmanfilter = KalmanFilterUWB(self.q)
        self.current_velocities = []
        self.current_measurement = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.current_measurement_UWB = []
        self.eps = 0.1  # 0.08

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
        if self.start_measure:
            self.current_measurement_UWB = self.mqtt.get_pos()
            print(self.current_measurement_UWB)
            self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posz']/-100] + self.current_measurement_UWB
            self.current_velocities = [self.mambo.sensors.speed_x,
                                       self.mambo.sensors.speed_y,
                                       self.mambo.sensors.speed_z]
            # self.current_state = self.kalmanfilter.get_state_estimation(
            #     self.q, self.current_velocities, self.current_measurement, self.p, True)
            # self.current_state = self.kalmanfilter.get_state_estimate(self.current_measurement,
            #                                                           self.current_velocities)
            self.controller.set_current_state(self.current_state)

    def go_to_xyz(self, desired_state):
        self.desired_state = desired_state
        self.controller.set_desired_state(self.desired_state)
        distance = ((self.current_state[0] - self.desired_state[0])**2 +
                    (self.current_state[1] - self.desired_state[1])**2 +
                    (self.current_state[2] - self.desired_state[2])**2)**0.5
        while distance > self.eps:
            cmd = self.controller.calculate_cmd_input()
            self.mambo.fly_direct(roll=cmd[0],
                                  pitch=cmd[1],
                                  yaw=cmd[2],
                                  vertical_movement=cmd[3],
                                  duration=None)
            self.mambo.smart_sleep(0.5)
            distance = ((self.current_state[0] - self.desired_state[0])**2 +
                        (self.current_state[1] - self.desired_state[1])**2 +
                        (self.current_state[2] - self.desired_state[2])**2)**0.5
            print(f"current state >>{self.current_state}")
            # print(f"cmd >> {cmd}")
            print(f"distance >> {distance}")

if __name__ == '__main__':
    mambo1 = Drone("84:20:96:91:73:F1")



# "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
