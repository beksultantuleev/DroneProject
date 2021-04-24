from Drone import Drone
from controllers.LQRcontroller import LQRcontroller
from controllers.PIDcontroller import PIDcontroller
import numpy as np
import time
from filters.KalmanFilterUWB import KalmanFilterUWB
from makeLogs.BlackBoxGenerator import Logger


class ModelBasedAgent(Drone):
    def __init__(self, drone_mac, use_wifi, controller, start_loggin=True):
        super().__init__(drone_mac, use_wifi)
        # ==================Kalman setup
        self.p = np.zeros((3, 3))
        self.q = np.zeros((3, 1))
        self.kalmanfilterUWB = KalmanFilterUWB(self.q)
        # ================== Controller setup
        self.controller = controller.lower()
        if self.controller == "lqr":
            self.title = "ModelBasedAgentLQR"
            self.controller = LQRcontroller()
        elif self.controller == "pid":
            self.title = "ModelBasedAgentPID"
            self.controller = PIDcontroller()
        else:
            raise ValueError("NO such controller found")
        # ==================
        self.start_loggin = start_loggin
        self.current_velocities = []
        self.current_measurement = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.eps = 0.15  # 0.08
        self.start_measure = False
        self.black_box = Logger()
        self.duration = None
        self.initialTime = None

    def sensor_callback(self, args):
        if self.start_measure:
            self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]
            self.current_velocities = [self.mambo.sensors.speed_x,
                                       self.mambo.sensors.speed_y,
                                       self.mambo.sensors.speed_z]

            self.p, self.q = self.kalmanfilterUWB.get_state_estimation(
                self.q, self.current_velocities, self.current_measurement, self.p, True)
            self.current_state = self.q.T.tolist()[0]
            self.controller.set_current_state(self.current_state)
            # print(f'current meas >> {self.current_measurement}')
            # print(f'current state {self.current_state}')

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
                    # self.mambo.smart_sleep(0.1)
                self.start_measure = True
                # self.mambo.smart_sleep(0.2) #istead of time sleep
                if self.use_wifi:
                    print(f'getting first state...>>{self.current_state}')
                    while self.current_state == []:
                        continue
                else:
                    print(f'getting first state...>>{self.current_state}')
                    while self.current_state == []:
                        self.mambo.smart_sleep(0.1)
                        print(f'current state in WHILE>>{self.current_state}')

                print(f'first state AFTER while>>{self.current_state}')

                '''after this function you need to feed action function such as go to xyz '''

    def go_to_xyz(self, desired_state):
        self.desired_state = desired_state
        self.initialTime = time.time()
        self.controller.set_desired_state(self.desired_state)
        distance = ((self.current_state[0] - self.desired_state[0])**2 +
                    (self.current_state[1] - self.desired_state[1])**2 +
                    (self.current_state[2] - self.desired_state[2])**2)**0.5
        while distance > self.eps:
            cmd = self.controller.calculate_cmd_input()
            if self.use_wifi == False:
                self.duration = 0.5
            self.mambo.fly_direct(roll=cmd[0],
                                  pitch=cmd[1],
                                  yaw=cmd[2],
                                  vertical_movement=cmd[3],
                                  duration=self.duration)

            distance = ((self.current_state[0] - self.desired_state[0])**2 +
                        (self.current_state[1] - self.desired_state[1])**2 +
                        (self.current_state[2] - self.desired_state[2])**2)**0.5
            # logging in thread
            if self.start_loggin:
                self.black_box.start_logging(["IMU", self.current_measurement], [
                    "Kalman", self.current_state], ["CMD", cmd], ["Distance", [distance]], ["Time", [np.round((time.time()-self.initialTime), 1)]], ["Title", [self.title]])
            print("===============================Start")
            print(f"logging >> {self.start_loggin}")
            print(f"controller >> {self.title}")
            print(f"current measurement >>{self.current_measurement}")
            print(f"KALMAN STATE >>{self.current_state}")
            print(f"CMD input >> {cmd}")
            print(f"desired state >> {self.desired_state}")
            print(f"distance left >> {distance}")


if __name__ == "__main__":
    mambo1 = "D0:3A:49:F7:E6:22"
    mambo2 = "D0:3A:0B:C5:E6:22"
    mambo3 = "D0:3A:B1:DC:E6:20"
    drone1 = ModelBasedAgent(mambo1, False, "pid", False)
    drone1.start_and_prepare()
    # drone1.mambo.turn_degrees(180)
    drone1.go_to_xyz([1, 0, 1])
    drone1.land_and_disconnect()

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
# "84:20:96:6c:22:67"
