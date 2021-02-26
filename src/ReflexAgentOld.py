from Drone import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman
import numpy as np


class ReflexAgent(Drone):

    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.current_measurement = []
        self.desired_state = []
        self.start_measure = False
        self.current_state = []

    def sensor_callback(self, args):
        if self.start_measure:
            self.current_measurement = list(np.around(np.array([self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                                self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                                self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]), 3))

    def fly_direct_fixed(self):
        self.mambo.fly_direct(0, 20, 0, 0, None)

    def go_to_xyz(self, desired_state):
        # sensor scale in pos X  #move forward and backward, headlight of drone +
        # sensor scale in pos Y #move sideways, right is +
        # sensor in pos Z  # move up and down, move up is -
        self.desired_state = desired_state

        stop_value_x = desired_state[0]  # x
        stop_value_y = desired_state[1]  # y
        stop_value_z = desired_state[2]  # z

        # get initial positions
        pos_x = self.current_measurement[0]
        pos_y = self.current_measurement[1]
        pos_z = self.current_measurement[2]

        if desired_state[0] == desired_state[1] == desired_state[2] == 0:
            print("i am already here")

        # move forward and backward
        if desired_state[0] > 0:
            while pos_x < stop_value_x:
                self.fly_direct_fixed()
                self.mambo.smart_sleep(0.5)
                pos_x = self.current_measurement[0]
                print(f"pos x {pos_x}")

        elif desired_state[0] < 0:
            while pos_x > stop_value_x:
                self.mambo.fly_direct(0, -20, 0, 1)
                self.mambo.smart_sleep(0.5)
                pos_x = self.current_measurement[0]
                print(f"pos x {pos_x}")

        # move sideways
        if desired_state[1] > 0:
            self.mambo.turn_degrees(90)
            while pos_y < stop_value_y:
                self.fly_direct_fixed()
                self.mambo.smart_sleep(0.5)
                pos_y = self.current_measurement[1]
                print(f"pos y {pos_y}")
            self.mambo.turn_degrees(-90)
        elif desired_state[1] < 0:
            self.mambo.turn_degrees(-90)
            self.mambo.smart_sleep(0.5)
            while pos_y > stop_value_y:
                self.fly_direct_fixed()
                self.mambo.smart_sleep(0.5)
                pos_y = self.current_measurement[1]
                print(f"pos y {pos_y}")

            self.mambo.turn_degrees(90)

        # move up and down
        if desired_state[2] > 0:
            while pos_z > stop_value_z:
                self.mambo.fly_direct(0, 0, 30, 1)
                self.mambo.smart_sleep(0.5)
                pos_z = self.current_measurement[2]
                print(f"pos z {pos_z}")

        elif desired_state[2] < 0:
            while pos_z < stop_value_z:
                self.mambo.fly_direct(0, 0, -30, 1)
                self.mambo.smart_sleep(0.5)
                pos_z = self.current_measurement[2]
                print(f"pos z {pos_z}")



if __name__ == "__main__":
    # detection_drone = ReflexAgent("7A:64:62:66:4B:67") #"84:20:96:6c:22:67" #"84:20:96:91:73:F1"
    detection_drone = ReflexAgent("84:20:96:91:73:F1")
    detection_drone.start_and_prepare()
    detection_drone.go_to_xyz([1,0,1])
    detection_drone.land_and_disconnect()
