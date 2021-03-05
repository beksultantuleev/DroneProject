from Drone import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman
import numpy as np
# ==============
from KalmanFilterUWB import KalmanFilterUWB
import time
from PIDcontroller import PIDcontroller

class ModelBasedAgent(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        self.controllerPID = PIDcontroller()
        self.kalmanfilter = MamboKalman([0, 0, 0], [0, 0, 0])
        # =======================
        self.p = np.zeros((3, 3))
        self.q = np.zeros((3, 1))
        self.kalmanfilterUWB = KalmanFilterUWB(self.q)
        # ==================
        self.current_velocities = []
        self.current_measurement = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.eps = 0.2  # 0.08
        self.start_measure = False
        self.current_state_oldKalman = []

    def sensor_callback(self, args):
        if self.start_measure:
            self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]
            self.current_velocities = [self.mambo.sensors.speed_x,
                                       self.mambo.sensors.speed_y,
                                       self.mambo.sensors.speed_z]
            self.current_state_oldKalman = self.kalmanfilter.get_state_estimate(self.current_measurement,
                                                                      self.current_velocities)
            # ========================
            self.p, self.q = self.kalmanfilterUWB.get_state_estimation(
                self.q, self.current_velocities, self.current_measurement, self.p, True)
            self.current_state = self.q.T.tolist()[0]
            # print(f"current state >>>{self.current_state}")
            # =======================
            # self.controller.set_current_state(self.current_state)
            self.controllerPID.set_current_state(self.current_state)

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
            self.mambo.safe_takeoff(2)  # we have extended from 3 to 10

            if self.mambo.sensors.flying_state != 'emergency':

                print('Sensor calibration...')
                while self.mambo.sensors.speed_ts == 0:
                    continue
                self.start_measure = True
                time.sleep(0.2)
                print('getting first state')
                while self.current_state == []:
                    continue
                '''after this function you need to feed action function such as go to xyz '''

    def go_to_xyz(self, desired_state):
        self.desired_state = desired_state
        # self.controller.set_desired_state(self.desired_state)
        self.controllerPID.set_desired_state(self.desired_state)
        distance = ((self.current_state[0] - self.desired_state[0])**2 +
                    (self.current_state[1] - self.desired_state[1])**2 +
                    (self.current_state[2] - self.desired_state[2])**2)**0.5
        while distance > self.eps:
            # cmd = self.controller.calculate_cmd_input()
            cmd = self.controllerPID.calculate_cmd_input()
            self.mambo.fly_direct(roll=cmd[0],
                                  pitch=cmd[1],
                                  yaw=cmd[2],
                                  vertical_movement=cmd[3],
                                  duration=None)
            # self.mambo.smart_sleep(0.3)
            time.sleep(0.3)
            distance = ((self.current_state[0] - self.desired_state[0])**2 +
                        (self.current_state[1] - self.desired_state[1])**2 +
                        (self.current_state[2] - self.desired_state[2])**2)**0.5
            print(f"NEW Kalman current state >>{self.current_state}")
            print(f"OLD Kalman current state >>{self.current_state_oldKalman}")
            print(f"cmd >> {cmd}")
            print(f"distance >> {distance}")


if __name__ == "__main__":
    modelAgent = ModelBasedAgent("84:20:96:91:73:F1")
    # modelAgent = ModelBasedAgent("7A:64:62:66:4B:67")
    modelAgent.start_and_prepare()
    modelAgent.go_to_xyz([1, 1, 1])
    modelAgent.land_and_disconnect()

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
