from Drone import Drone
from PositionController import MamboPositionController
# from KalmanFilterUWB import KalmanFilterUWB
import numpy as np
# from multiprocessing import Process,Queue,Pipe
# from UWBscript_modified import *
from DroneUWBtest import DroneUWB

class ModelBasedAgentUWB(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        # self.kalmanfilter = MamboKalman([0, 0, 0], [0, 0, 0])
        self.q = np.ones((3, 1))
        self.p = p = np.zeros((3, 3))
        # self.kalmanfilter = KalmanFilterUWB(self.q)
        self.uwbPosition = DroneUWB()
        self.current_velocities = []
        self.current_measurement = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.current_measurement_UWB = []
        self.eps = 0.1  # 0.08
        self.start_measure = False
        #________________________UWB
        # self.parent_conn, self.child_conn = Pipe() #added connections
        # self.proccess = Process(target=runUWB, args=(self.child_conn,))
        #__________________________

    def sensor_callback(self, args):
        if self.start_measure:
            self.uwbPosition.mqtt_connection()
            self.current_measurement_UWB = self.uwbPosition.pos
            #UWB_______________________
            # self.proccess.start() #starting process
            # self.current_measurement_UWB = self.uwbPosition.get_pos()  # populate with subscription
            print(self.current_measurement_UWB)
            #UWB________________________
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


if __name__ == "__main__":
    modelAgent = ModelBasedAgentUWB("84:20:96:91:73:F1")
    modelAgent.start_and_prepare()
    print("hello")
    modelAgent.mambo.fly_direct(0,30,0,0,1)

    # modelAgent.start_and_prepare()
    # waypoint = [[1,0,1],[2,0,1]]
    # for i in waypoint:
    #     modelAgent.go_to_xyz(i)
    modelAgent.land_and_disconnect()

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
