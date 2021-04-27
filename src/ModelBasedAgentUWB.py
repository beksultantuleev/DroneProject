from Drone import Drone
from controllers.LQRcontroller import LQRcontroller
from controllers.PIDcontroller import PIDcontroller
from filters.KalmanFilterUWB import KalmanFilterUWB
import numpy as np
from UWB_subscriber.subscriber import MqttSubscriber
import time
from makeLogs.BlackBoxGenerator import Logger


class ModelBasedAgentUWB(Drone):
    def __init__(self, drone_mac, use_wifi, controller, local, start_loggin=True, topic = "Position3"):
        super().__init__(drone_mac, use_wifi)
        
        self.topic = topic
        self.local = local
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
        # ===================Kalman setup
        self.p = np.zeros((3, 3))
        self.q = np.zeros((3, 1))
        self.kalmanfilterUWB = KalmanFilterUWB(self.q)
        # ==================
        self.current_velocities = []
        self.current_measurement_IMU = []
        self.current_measurement_IMU_global = []
        self.current_measurement_combined = []
        self.current_state = []  # meters
        self.desired_state = []  # meters
        self.current_measurement_UWB = []
        self.eps = 0.15  # it was 0.2
        self.start_measure = False
        self.black_box = Logger()
        # ================
        self.mqttSubscriber = MqttSubscriber(
            "192.168.1.200", 1883, self.topic)
        self.mqttSubscriber.start()
        self.UWB_positions = []
        self.initial_pos = []
        self.initial_pos_bool = True
        self.current_measurement_combined = []
        self.rotation_matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])
        self.initialTime = None
        self.duration = None
        self.start_loggin = start_loggin
        #

    def get_avrg_uwb_pos(self, number):
        # print( self.mqttSubscriber.pos)
        a = self.mqttSubscriber.pos
        for i in range(number):
            if a:
                self.UWB_positions.append(a)
        if not len(self.UWB_positions) == 0:
            return (list(np.mean(self.UWB_positions, axis=0)))

    def sensor_callback(self, args):
        if self.get_avrg_uwb_pos(30) and self.initial_pos_bool:
            self.initial_pos = self.get_avrg_uwb_pos(30)
            self.initial_pos_bool = False
        if self.start_measure:
            if self.local:
                "in xyz, where x is north, 1x3"
                self.current_measurement_IMU = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                                self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                                self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]

                "in xyz, where y is north, + this make it in local scale, 1x3"
                self.current_measurement_UWB = list(
                    np.array(self.mqttSubscriber.pos) - np.array(self.initial_pos))

                "in xyz, where x is north, 1x3"
                self.current_velocities = [self.mambo.sensors.speed_x,
                                           self.mambo.sensors.speed_y,
                                           self.mambo.sensors.speed_z]
                "for kalman filter i want xyz(imu) + rotated(xyz_uwb)"
                self.current_measurement_combined = self.current_measurement_IMU + \
                    list(np.dot(self.rotation_matrix, self.current_measurement_UWB))

            else:
                # global
                "in xyz, where x is north, 1x3"
                self.current_measurement_IMU = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                                self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                                self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]

                "in xyz, where x is north, + it makes IMU + initial UWB, 1x3"
                self.current_measurement_IMU_global = list(np.array(
                    self.current_measurement_IMU) + np.dot(self.rotation_matrix,  [self.initial_pos[0], self.initial_pos[1], self.current_measurement_IMU[2]]))  # mod here? instead of just self.initial_pos

                "in xyz, where x is north, in raw data, 1x3"
                self.current_measurement_UWB = list(
                    np.dot(self.rotation_matrix, self.mqttSubscriber.pos))

                "in xyz, where x is north"
                self.current_velocities = [self.mambo.sensors.speed_x,
                                           self.mambo.sensors.speed_y,
                                           self.mambo.sensors.speed_z]
                "for kalman i want global_imu+ uwb 1x6"
                self.current_measurement_combined = self.current_measurement_IMU_global + \
                    self.current_measurement_UWB

            "then we put combined locations into kalman filter"
            self.p, self.q = self.kalmanfilterUWB.get_state_estimation(
                self.q, self.current_velocities, self.current_measurement_combined, self.p, True)
            self.current_state = self.q.T.tolist()[0]
            self.controller.set_current_state(self.current_state)

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
                if self.use_wifi:
                    print('getting first state')
                    while self.current_state == []:
                        continue
                else:
                    print(f'getting first state...>>{self.current_state}')
                    while self.current_state == []:
                        self.mambo.smart_sleep(0.1)
                        print(f'current state in WHILE>>{self.current_state}')
                '''after this function you need to feed action function such as go to xyz '''

    def go_to_xyz(self, desired_state):
        if self.local:  # local
            self.desired_state = desired_state
        else:  # global
            self.desired_state = list(
                np.dot(self.rotation_matrix, desired_state)) 

        self.initialTime = time.time()
        self.controller.set_desired_state(self.desired_state)
        distance = ((self.current_state[0] - self.desired_state[0])**2 +
                    (self.current_state[1] - self.desired_state[1])**2 +
                    (self.current_state[2] - self.desired_state[2])**2)**0.5
        while distance > self.eps:
            cmd = self.controller.calculate_cmd_input()
            if self.use_wifi == False:
                if distance>2:
                    self.duration = 1
                
                #try to make uav fly longer if it apploaches to desired state
                elif distance <0.2:
                    self.duration =1
                else:
                    self.duration = 0.5
            self.mambo.fly_direct(roll=cmd[0],
                                  pitch=cmd[1],
                                  yaw=cmd[2],
                                  vertical_movement=cmd[3],
                                  duration=self.duration)
            distance = ((self.current_state[0] - self.desired_state[0])**2 +
                        (self.current_state[1] - self.desired_state[1])**2)**0.5  # + (self.current_state[2] - self.desired_state[2])**2

            # logging
            if self.start_loggin:
                self.black_box.start_logging(["IMU", self.current_measurement_IMU], [
                    "Kalman", self.current_state], ["UWB", self.current_measurement_UWB], ["Distance", [distance]], ["Time", [np.round((time.time()-self.initialTime), 1)]], ["Title", [self.title]], ["Local", [self.local]])

            print("===============================Start")
            print(f"controller >> {self.title}")
            print(f" Local >>> {self.local}. logging >> {self.start_loggin}")         
            print(f"initial pos (avrg) {self.initial_pos}")
            print(f"UWB >>{self.current_measurement_UWB}")
            print(f"KALMAN STATE >>{self.current_state}")
            print(f"Desired state >>{self.desired_state}")
            print(f"CMD input >> {cmd}")
            print(f"distance >> {distance}")
            # print(
            #     f"current meas_combined>>{self.current_measurement_combined}")


    def land_and_disconnect(self):
        print('Landing...')
        self.mambo.safe_land(3)
        self.mambo.smart_sleep(2)
        print('Disconnecting...')
        self.mambo.disconnect()
        self.mqttSubscriber.stop()


if __name__ == "__main__":
    mambo1 = "D0:3A:49:F7:E6:22"
    mambo2 = "D0:3A:0B:C5:E6:22" #new tag
    mambo3 = "D0:3A:B1:DC:E6:20" #with no sticker
    modelAgent = ModelBasedAgentUWB(
        mambo3, use_wifi=False, controller="pid", local=True, topic="Position1")
    modelAgent.start_and_prepare()

    modelAgent.go_to_xyz([1, 0, 1])
    # modelAgent.go_to_xyz([2.5, 3.6, 1])
    # modelAgent.go_to_xyz([2.5, 6, 1])
    modelAgent.land_and_disconnect()

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
# "84:20:96:6c:22:67"
