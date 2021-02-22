from Drone_main import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman
import numpy as np


class ModelBasedAgent(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        self.kalmanfilter = MamboKalman([0, 0, 0], [0, 0, 0])
        # for use with KalmanFilter (used as u (input))
        self.current_velocities = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.eps = 0.3  # 0.08
        self.start_measure = False

    def sensor_callback(self, args):
        if self.start_measure:
            self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]
            self.current_velocities = [self.mambo.sensors.speed_x,
                                       self.mambo.sensors.speed_y,
                                       self.mambo.sensors.speed_z]
            self.current_state = self.kalmanfilter.get_state_estimate(self.current_measurement,
                                                                      self.current_velocities)
            self.controller.set_current_state(self.current_state)

    def go_to_xyz(self, desired_state):
        self.desired_state = desired_state

        stop_value_x = desired_state[0]  # x
        stop_value_y = desired_state[1]  # y
        stop_value_z = desired_state[2]  # z

        # get initial positions
        pos_x = self.current_state[0]
        pos_y = self.current_state[1]
        pos_z = self.current_state[2]

        if stop_value_x > 0:
            self.controller.set_desired_state([stop_value_x, 0, stop_value_z])
            cmd = self.controller.calculate_cmd_input()
            while pos_x < stop_value_x:
                if stop_value_z == 0:
                    self.mambo.fly_direct(roll=cmd[0],
                                          pitch=cmd[1],
                                          yaw=cmd[2],
                                          vertical_movement=0,
                                          duration=None)
                    self.mambo.smart_sleep(0.1)
                else:
                    self.mambo.fly_direct(roll=cmd[0],
                                          pitch=cmd[1],
                                          yaw=cmd[2],
                                          vertical_movement=cmd[3],
                                          duration=None)
                    self.mambo.smart_sleep(0.1)
                pos_x = self.current_state[0]
                print(f"cmd >>{cmd}")
                print(f"pos x {pos_x}")
        elif stop_value_x < 0:
            self.controller.set_desired_state([stop_value_x, 0, stop_value_z])
            cmd = self.controller.calculate_cmd_input()
            while pos_x > stop_value_x:
                if stop_value_z == 0:
                    self.mambo.fly_direct(roll=cmd[0],
                                          pitch=cmd[1],
                                          yaw=cmd[2],
                                          vertical_movement=0,
                                          duration=None)
                    self.mambo.smart_sleep(0.1)
                else:
                    self.mambo.fly_direct(roll=cmd[0],
                                          pitch=cmd[1],
                                          yaw=cmd[2],
                                          vertical_movement=cmd[3],
                                          duration=None)
                    self.mambo.smart_sleep(0.1)
                pos_x = self.current_state[0]
                print(f"cmd >>{cmd}")
                print(f"pos x {pos_x}")

        self.mambo.smart_sleep(1)

        if stop_value_y > 0:
            self.controller.set_desired_state(self.desired_state)
            cmd = self.controller.calculate_cmd_input()
            while pos_y < stop_value_y:
                self.mambo.fly_direct(roll=cmd[0],
                                      pitch=cmd[1],
                                      yaw=cmd[2],
                                      vertical_movement=cmd[3],
                                      duration=None)
                self.mambo.smart_sleep(0.1)
                pos_y = self.current_state[1]
                print(f"cmd >>{cmd}")
                print(f"pos y {pos_y}")
        elif stop_value_y < 0:
            self.controller.set_desired_state(self.desired_state)
            cmd = self.controller.calculate_cmd_input()
            while pos_y > stop_value_y:
                self.mambo.fly_direct(roll=cmd[0],
                                      pitch=cmd[1],
                                      yaw=cmd[2],
                                      vertical_movement=cmd[3],
                                      duration=None)
                self.mambo.smart_sleep(0.1)
                pos_y = self.current_state[1]
                print(f"cmd >>{cmd}")
                print(f"pos y {pos_y}")

    def go_to_xyz_imp(self, desired_state):
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
            print(f"cmd >> {cmd}")
            print(f"distance >> {distance}")

    def flight_function(self, args):
        if self.mambo.sensors.flying_state != 'emergency':

            print('Sensor calibration...')
            while self.mambo.sensors.speed_ts == 0:
                continue
            self.start_measure = True

            print('getting first state')
            while self.current_state == []:
                continue
            '''run the function here'''
            # self.go_to_xyz([1, 1, 0])
            self.go_to_xyz_imp([1, 0, 1])

        print('landing')
        self.mambo.safe_land(3)


if __name__ == "__main__":
    modelAgent = ModelBasedAgent("84:20:96:91:73:F1")
    modelAgent.run()

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
