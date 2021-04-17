from math import nan
from operator import mod
from numpy.core.numeric import NaN
from Drone import Drone
from controllers.LQRcontroller import LQRcontroller
from controllers.PIDcontroller import PIDcontroller
from filters.KalmanFilterUWB import KalmanFilterUWB
import numpy as np
from UWB_subscriber.subscriber import MqttSubscriber
import time
from makeLogs.BlackBoxGenerator import Logger
import threading

# from subscriber import MqttSubscriber


class ModelBasedAgentUWB(Drone):
    def __init__(self, drone_mac, use_wifi, controller):
        super().__init__(drone_mac, use_wifi)
        # ================== Controller setup
        self.controller = controller.lower()
        if self.controller == "lqr":
            self.title = "ModelBasedAgendUWB_LQR"
            self.controller = LQRcontroller()
        elif self.controller == "pid":
            self.title = "ModelBasedAgendUWB_PID"
            self.controller = PIDcontroller()
        else:
            raise ValueError("NO such controller found")
        # ==================
        # =======================
        self.p = np.zeros((3, 3))
        self.q = np.zeros((3, 1))
        self.kalmanfilterUWB = KalmanFilterUWB(self.q)
        # ==================
        self.current_velocities = []
        self.current_measurement = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.current_state_UWB = []
        self.eps = 0.35  # 0.08
        self.start_measure = False
        self.black_box = Logger()
        # ================
        self.mqttSubscriber = MqttSubscriber(
            "192.168.1.200", 1883, "Position3")
        self.mqttSubscriber.start()
        self.UWB_positions = []
        self.initial_pos = []
        self.initial_pos_bool = True
        self.current_measurement_combined = []
        self.rotation_matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])
        self.initialTime = None

    def getUWB_avrg_pos(self):
        # print( self.mqttSubscriber.pos)
        a = self.mqttSubscriber.pos
        for i in range(100):
            if a:
                self.UWB_positions.append(a)
        if not len(self.UWB_positions) == 0:
            return (list(np.mean(self.UWB_positions, axis=0)))

    def UWB_position_filter(self, given_position):
        time.sleep(0.1)
        self.new_position = list(self.mqttSubscriber.pos)
        difference = ((self.new_position[0] - given_position[0])**2 + (
            self.new_position[1] - given_position[1])**2)**0. # +(self.new_position[2] - given_position[2])**2
        # print(f"difference >>{difference}")
        # print(f"given_pos >>{given_position}")
        # print(f"new_position>> {self.new_position}")
        if difference > 0.5:
            return given_position
        time.sleep(0.1)
        return self.new_position

    def sensor_callback(self, args):
        if self.getUWB_avrg_pos() and self.initial_pos_bool:
            self.initial_pos = self.getUWB_avrg_pos()
            self.initial_pos_bool = False
        if self.start_measure:
            self.current_measurement = list(np.array([self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                                      self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                                      self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]) + np.array([self.initial_pos[0], self.initial_pos[1], 0]))
            self.current_velocities = [self.mambo.sensors.speed_x,
                                       self.mambo.sensors.speed_y,
                                       self.mambo.sensors.speed_z]
            self.current_measurement_combined = self.current_measurement + list(self.mqttSubscriber.pos)
                #list(self.mqttSubscriber.pos)  # self.getUWB_avrg_pos()

            self.p, self.q = self.kalmanfilterUWB.get_state_estimation(
                self.q, self.current_velocities, self.current_measurement_combined, self.p, True)
            self.current_state = self.q.T.tolist()[0]
            self.current_state = list(np.dot(self.rotation_matrix, self.current_state))
            self.controllerLQR.set_current_state(self.current_state)
            # print(f"UWB real >>{self.mqttSubscriber.pos}")
            # print(f"UWB avrg >>{self.getUWB_avrg_pos()}")
            # print(f"current meas>>{self.current_measurement}")
            # print(f"current meas_combined>>{self.current_measurement_combined}")
            # print(f"kalman>>{self.current_state}")
            # print(f"velocity >>{self.current_velocities}")

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
                # self.mambo.smart_sleep(0.2) #istead of time sleep
                # time.sleep(0.2)
                print('getting first state')
                while self.current_state == []:
                    continue
                '''after this function you need to feed action function such as go to xyz '''

    def go_to_xyz(self, desired_state):
        self.desired_state = desired_state#list(np.dot(self.rotation_matrix, desired_state)) #rotate positions
        self.initialTime = time.time()
        self.controllerLQR.set_desired_state(self.desired_state)
        distance = ((self.current_state[0] - self.desired_state[0])**2 +
                    (self.current_state[1] - self.desired_state[1])**2 +
                    (self.current_state[2] - self.desired_state[2])**2)**0.5  
        while distance > self.eps:
            cmd = self.controllerLQR.calculate_cmd_input()
            self.mambo.fly_direct(roll=cmd[1],
                                  pitch=cmd[0],
                                  yaw=cmd[2],
                                  vertical_movement=cmd[3],
                                  duration=None)
            time.sleep(0.25)
            distance = ((self.current_state[0] - self.desired_state[0])**2 +
                        (self.current_state[1] - self.desired_state[1])**2 )**0.5 #+ (self.current_state[2] - self.desired_state[2])**2
            
            # logging
            thread = threading.Thread(target=self.black_box.start_logging(["IMU", self.current_measurement], [
                "Kalman", self.current_state], ["UWB", list(self.mqttSubscriber.pos)], ["Distance", [distance]], ["Time", [np.round((time.time()-self.initialTime), 1)]], ["Title", [self.title]]))
            thread.start()
            print("===============================Start")
            print(f"UWB >>{list(self.mqttSubscriber.pos)}")
            print(f"current meas_combined>>{self.current_measurement_combined}")
            print(f"initial pos (avrg) {self.initial_pos}")
            print(f"KALMAN STATE >>{self.current_state}")           
            print(f"Desired state >>{self.desired_state}")
            print(f"CMD input >> {cmd}")
            print(f"distance >> {distance}")
            # print("===============================end")

    def land_and_disconnect(self):
        print('Landing...')
        self.mambo.safe_land(3)
        self.mambo.smart_sleep(2)
        print('Disconnecting...')
        self.mambo.disconnect()
        self.mqttSubscriber.stop()


if __name__ == "__main__":
    modelAgent = ModelBasedAgentUWB("84:20:96:6c:22:67", True)
    # modelAgent = ModelBasedAgent("7A:64:62:66:4B:67")
    modelAgent.start_and_prepare()
# modelAgent.mambo.hover()
    # modelAgent.mambo.smart_sleep(3)
    # black tape [2.5, 3.5, 1] #white tape [2.2, 1.7.1]
    modelAgent.go_to_xyz([2.5, 3.65, 1])

    modelAgent.land_and_disconnect()

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
# "84:20:96:6c:22:67"
