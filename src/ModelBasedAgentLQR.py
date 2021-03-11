from Drone import Drone
from LQRcontroller import LQRcontroller
import numpy as np
import time
from KalmanFilterUWB import KalmanFilterUWB
from BlackBoxGenerator import Logger
import threading


class ModelBasedAgent(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controllerLQR = LQRcontroller()
        # self.kalmanfilter = MamboKalman([0, 0, 0], [0, 0, 0])
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
        self.black_box = Logger()

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
            self.controllerLQR.set_current_state(self.current_state)

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
        self.desired_state = desired_state
        self.controllerLQR.set_desired_state(self.desired_state)
        distance = ((self.current_state[0] - self.desired_state[0])**2 +
                    (self.current_state[1] - self.desired_state[1])**2 +
                    (self.current_state[2] - self.desired_state[2])**2)**0.5
        while distance > self.eps:
            cmd = self.controllerLQR.calculate_cmd_input()
            self.mambo.fly_direct(roll=cmd[0],
                                  pitch=cmd[1],
                                  yaw=cmd[2],
                                  vertical_movement=cmd[3],
                                  duration=None)
            distance = ((self.current_state[0] - self.desired_state[0])**2 +
                        (self.current_state[1] - self.desired_state[1])**2 +
                        (self.current_state[2] - self.desired_state[2])**2)**0.5
            #logging
            thread = threading.Thread(target=self.black_box.start_logging(["IMU", self.current_measurement], [
                                         "Kalman", self.current_state], ["CMD", cmd],["Distance", [distance]] , ["Time", [time.time()]]))
            thread.start()
            print(f"KALMAN STATE >>{self.current_state}")
            print(f"current measurement >>{self.current_measurement}")
            print(f"CMD input >> {cmd}")
            print(f"distance >> {distance}")


if __name__ == "__main__":
    modelAgent = ModelBasedAgent("84:20:96:6c:22:67")
    # modelAgent = ModelBasedAgent("7A:64:62:66:4B:67")
    modelAgent.start_and_prepare()

    modelAgent.go_to_xyz([1, 0, 1])
    # modelAgent.mambo.senso
    # modelAgent.mambo.smart_sleep(10)

    modelAgent.land_and_disconnect()

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
#"84:20:96:6c:22:67"