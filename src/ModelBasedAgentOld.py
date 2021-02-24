from Drone_main import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman
import numpy as np
waypoint = [[],[],[],[]]
wypoint.pop(0)
class ModelBasedAgent(Drone):

    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        self.kalmanfilter = MamboKalman([0, 0, 1], [0, 0, 0])  # pos and vel
        self.current_measurement = []
        self.current_state = []
        self.desired_state = []

    def sensor_callback(self, args):
        self.current_measurement = list(np.around(np.array([self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                                            self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                                            self.mambo.sensors.sensors_dict['DronePosition_posz']/-100], dtype=float), 3))
        self.current_velocity = list(np.around(np.array([self.mambo.sensors.speed_x,
                                                         self.mambo.sensors.speed_y, self.mambo.sensors.speed_z]), 3))
        self.current_state = list(np.around(np.array(self.kalmanfilter.get_state_estimate(
            self.current_measurement, self.current_velocity)), 3))

        '''you can compare kalman vs non kalman estimators'''
        # print(f" kalman filter>> {self.current_state}")
        # print(f"only sensors >> {self.current_measurement}")

    # def fly_with_position_controller(self):
    #     self.controller.set_current_state([0,0,0])
    #     self.controller.set_desired_state(self.desired_state)
    #     cmd = self.controller.calculate_cmd_input()
    #     return cmd

    def fly_direct_fixed(self):
        self.mambo.fly_direct(0, 20, 0, 0, None)  # 20 was good

    def go_to_xyz(self, desired_state):
        self.desired_state = desired_state

        stop_value_x = desired_state[0]  # x
        stop_value_y = desired_state[1]  # y
        stop_value_z = desired_state[2]  # z

        # get initial positions
        pos_x = self.current_state[0]
        pos_y = self.current_state[1]
        pos_z = self.current_state[2]

        if desired_state[0] == desired_state[1] == desired_state[2] == 0:
            print("i am already here")
        

        # move forward and backward
        if stop_value_x > 0:
            while pos_x < stop_value_x:
                self.fly_direct_fixed()
                self.mambo.smart_sleep(0.01)
                # self.mambo.fly_direct(self.fly_with_position_controller()[0], self.fly_with_position_controller()[1], 0, self.fly_with_position_controller()[3], None)
                pos_x = self.current_state[0]
                print(f"pos x {pos_x}")
                # print(self.fly_with_position_controller())

        elif stop_value_x < 0:
            while pos_x > stop_value_x:
                self.mambo.fly_direct(0, -20, 0, 1)
                self.mambo.smart_sleep(0.5)
                # self.mambo.fly_direct(self.fly_with_position_controller()[0], self.fly_with_position_controller()[1], 0, self.fly_with_position_controller()[3], 1)
                pos_x = self.current_state[0]
                print(f"pos x {pos_x}")
                # print(self.fly_with_position_controller())

        # move sideways
        if stop_value_y > 0:
            self.mambo.turn_degrees(90)
            while pos_y < stop_value_y:
                self.fly_direct_fixed()
                self.mambo.smart_sleep(0.5)
                # self.mambo.fly_direct(self.fly_with_position_controller()[0], self.fly_with_position_controller()[1], 0, self.fly_with_position_controller()[3], 1)
                pos_y = self.current_state[1]
                print(f"pos y {pos_y}")
                # print(self.fly_with_position_controller())
            self.mambo.turn_degrees(-90)
        elif stop_value_y < 0:
            self.mambo.turn_degrees(-90)
            self.mambo.smart_sleep(0.5)
            while pos_y > stop_value_y:
                self.fly_direct_fixed()
                self.mambo.smart_sleep(0.5)
                # self.mambo.fly_direct(self.fly_with_position_controller()[0], self.fly_with_position_controller()[1], 0, self.fly_with_position_controller()[3], 1)
                pos_y = self.current_state[1]
                print(f"pos y {pos_y}")
                # print(self.fly_with_position_controller())
            self.mambo.turn_degrees(90)

         # move up and down
        if stop_value_z > 0:
            while pos_z < stop_value_z:
                # print("pos z stop: %d" % stop_value_z)
                self.mambo.fly_direct(0,0,0,10,None)
                self.mambo.smart_sleep(0.1)
                # self.mambo.fly_direct(self.fly_with_position_controller()[0], self.fly_with_position_controller()[1], 0, self.fly_with_position_controller()[3], 1)
                pos_z = self.current_state[2]
                print("pos z: %f" % pos_z)

        elif stop_value_z < 0:
            while pos_z > stop_value_z:
                self.mambo.fly_direct(0,0,0,-30,None)
                self.mambo.smart_sleep(0.5)
                # self.mambo.fly_direct(self.fly_with_position_controller()[0], self.fly_with_position_controller()[1], 0, self.fly_with_position_controller()[3], 1)
                pos_z = self.current_state[2]
                print(f"pos z {pos_z}")


    def square_shape(self):
        self.go_to_xyz([1, 0, 1])
        self.go_to_xyz([0, 1, 1])
        self.go_to_xyz([-1, 0,1])
        self.go_to_xyz([0,-1, 1])

    def set_coordinates(self, x, y, z):
        self.go_to_xyz([x,y,z])

    def set_list_of_coordinates(self, aList):
        for i in aList:
            self.go_to_xyz(i)
    

    def flight_function(self, args):
        if self.mambo.sensors.flying_state != "emergency":
            print("Sensor Calibration...")
            while self.mambo.sensors.speed_ts == 0:
                continue
            # waypoints = [[1, 0, 0], [1, 1, 0]]
            self.go_to_xyz([1, 1, 0])
            # self.mambo.smart_sleep(2)
            # self.go_to_xyz([0, 0, 1])
            # self.square_shape()
            
            # self.square_shape()
            # self.set_coordinates(1,0,0)
            # self.set_list_of_coordinates([])
            # self.square_shape()

        else:
            print("not gonna fly")


if __name__ == "__main__":
    # detection_drone = ModelBasedAgent("7A:64:62:66:4B:67") #"84:20:96:6c:22:67" #"84:20:96:91:73:f1"
    detection_drone = ModelBasedAgent("84:20:96:91:73:F1")
    detection_drone.run()
