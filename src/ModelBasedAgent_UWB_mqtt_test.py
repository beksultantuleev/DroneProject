from Drone_mqtt_test import Drone
from PositionController import MamboPositionController
# from KalmanFilterUWB import KalmanFilterUWB
import numpy as np
# from multiprocessing import Process,Queue,Pipe
# from UWBscript_modified import *
import time

class ModelBasedAgentUWB(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        # self.kalmanfilter = MamboKalman([0, 0, 0], [0, 0, 0])
        self.q = np.ones((3, 1))
        self.p = p = np.zeros((3, 3))
        # self.kalmanfilter = KalmanFilterUWB(self.q)
        # self.Drone = Drone()
        self.current_velocities = []
        self.current_measurement = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.current_measurement_UWB = []
        self.eps = 0.1  # 0.08
        self.start_measure = False

    def sensor_callback(self, args):
        if self.start_measure:
            
            # self.current_measurement_UWB = self.pos
            # print(self.current_measurement_UWB)
            # print(self.pos)
            self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posz']/-100] #+ self.current_measurement_UWB
            self.current_velocities = [self.mambo.sensors.speed_x,
                                       self.mambo.sensors.speed_y,
                                       self.mambo.sensors.speed_z]
            # self.current_state = self.kalmanfilter.get_state_estimation(
            #     self.q, self.current_velocities, self.current_measurement, self.p, True)
            # self.current_state = self.kalmanfilter.get_state_estimate(self.current_measurement,
            #                                                           self.current_velocities)
            self.controller.set_current_state(self.current_state)
            print(self.current_measurement)

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
    # time.sleep(1)
    print("run mqtt") 
    # modelAgent.run()
    modelAgent.start_and_prepare()
    # modelAgent.mambo.hover()
    modelAgent.mambo.smart_sleep(1)
    # while True:
    #     time.sleep(0.5)
    #     print(modelAgent.pos)
    # modelAgent.start_and_prepare()
    # print("trying to fly direct")
    # modelAgent.mambo.fly_direct(0,10,0,0,1)
    # print("after fly direct")
    modelAgent.land_and_disconnect()
    # modelAgent.stop_client()
    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
