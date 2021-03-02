from Drone import Drone
from PositionController import MamboPositionController
from KalmanFilterUWB import KalmanFilterUWB
import numpy as np
from KalmanFilter import MamboKalman
# from multiprocessing import Process,Queue,Pipe
# from UWBscript_modified import *
# from DroneUWBtest import DroneUWB
from subscriber import MqttSubscriber
import time


class ModelBasedAgentUWB(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        # self.kalmanfilter = MamboKalman([0, 0, 0], [0, 0, 0])
        self.p = np.zeros((3, 3))
        self.q = np.ones((3, 1))
        self.kalmanfilter = KalmanFilterUWB(self.q)
        self.current_velocities = []
        self.current_measurement = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.eps = 0.2  # 0.08
        self.start_measure = False
        self.current_state_UWB = []
        self.mqttSubscriber = MqttSubscriber(
            "localhost", 1883, "Position2")
        self.mqttSubscriber.start()
        self.UWB_Data_Storing = True
        self.current_measurement_combined = []
        self.FLAG = False
        self.checker = False
        self.initial_pos = [0,0,0]

    def sensor_callback(self, args):
        if self.start_measure:
            if self.UWB_Data_Storing:
                print(self.UWB_Data_Storing)
                self.initial_pos = self.mqttSubscriber.pos
                self.UWB_Data_Storing = False

            self.current_state_UWB = self.mqttSubscriber.pos
            self.checker = self.mqttSubscriber.checker
            if self.checker:
                self.checker = False
                self.FLAG = True
            else:
                self.FLAG = False

            self.current_measurement = np.array([self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                                 self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                                 self.mambo.sensors.sensors_dict['DronePosition_posz']/-100] + np.array([self.initial_pos[0], self.initial_pos[1], 0]))
            self.current_measurement_combined = list(self.current_measurement) + list(self.mqttSubscriber.pos)
            # print(self.current_measurement_combined)
            self.current_velocities = [self.mambo.sensors.speed_x,
                                       self.mambo.sensors.speed_y,
                                       self.mambo.sensors.speed_z]
            self.p, self.q = self.kalmanfilter.get_state_estimation(
                self.q, self.current_velocities, self.current_measurement_combined, self.p, self.FLAG)
            # self.current_state = self.kalmanfilter.get_state_estimate(self.current_measurement,
            #                                                           self.current_velocities)
            self.controller.set_current_state(self.q.T.tolist()[0])
            print(f"updated Q>>>{self.q.T.tolist()[0]}")

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
            self.mambo.smart_sleep(0.01)
            distance = ((self.current_state[0] - self.desired_state[0])**2 +
                        (self.current_state[1] - self.desired_state[1])**2 +
                        (self.current_state[2] - self.desired_state[2])**2)**0.5
            print(f"current state >>{self.current_state}")
            print(f"cmd >> {cmd}")
            print(f"distance >> {distance}")


if __name__ == "__main__":
    modelAgent = ModelBasedAgentUWB("84:20:96:91:73:F1")
    # modelAgent = ModelBasedAgent("7A:64:62:66:4B:67")
    # while True:
    #     print(modelAgent.mqttSubscriber.pos)
    #     time.sleep(0.5)
    modelAgent.start_and_prepare()
    print("hello")
    # modelAgent.mambo.fly_direct(0,30,0,10,1) #test
    modelAgent.mambo.turn_degrees(90)
    # modelAgent.go_to_xyz([2.6, 4.7, 1])
    # modelAgent.mambo.smart_sleep(1)
    # modelAgent.go_to_xyz([2, 0, 1])

    modelAgent.land_and_disconnect()
    modelAgent.mqttSubscriber.stop()

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
