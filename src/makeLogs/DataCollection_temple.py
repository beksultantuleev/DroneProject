from Drone import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman
import numpy as np
import time
import logging

class DataCollection(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        self.kalmanfilter = MamboKalman([0, 0, 0], [0, 0, 0])
        self.current_velocities = []
        self.current_measurement = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.eps = 0.2  # 0.08
        self.start_measure = False
        logging.basicConfig(filename = 'position_log_fly_direct.csv', format='%(asctime)s, XYZ, %(message)s', level=logging.DEBUG)

    def sensor_callback(self, args):
        if self.start_measure:
            self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]
            self.current_velocities = [self.mambo.sensors.speed_x,
                                       self.mambo.sensors.speed_y,
                                       self.mambo.sensors.speed_z]
            # self.current_state = self.kalmanfilter.get_state_estimate(self.current_measurement,
            #                                                           self.current_velocities)
            # self.controller.set_current_state(self.current_state)
            logging.info(f"{self.current_measurement[0]}, {self.current_measurement[1]}, {self.current_measurement[2]}")
            # time.sleep(0.01)

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
            self.mambo.safe_takeoff(3)  # we have extended from 3 to 10

            if self.mambo.sensors.flying_state != 'emergency':

                print('Sensor calibration...')
                while self.mambo.sensors.speed_ts == 0:
                    continue
                self.start_measure = True
                time.sleep(0.2)
                print('getting first state')
                # while self.current_state == []:
                #     continue
                '''after this function you need to feed action function such as go to xyz '''



if __name__ == "__main__":
    modelAgent = DataCollection("84:20:96:91:73:F1")
    # modelAgent = ModelBasedAgent("7A:64:62:66:4B:67")
    modelAgent.start_and_prepare()
    modelAgent.mambo.fly_direct(0,15,0,0,2)
    modelAgent.mambo.turn_degrees(180)
    modelAgent.mambo.fly_direct(0,15,0,0,2)
    modelAgent.mambo.hover()

    modelAgent.land_and_disconnect()
