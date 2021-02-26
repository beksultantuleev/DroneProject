from numpy.core.fromnumeric import size
from Drone import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman
import numpy as np
import time


class PIDcontroller(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        self.kalmanfilter = MamboKalman([0, 0, 0], [0, 0, 0])
        self.current_velocities = []
        self.current_state = []  # meters
        self.desired_state = [2, 0, 1]  # meters #setpoint
        # self.eps = 0.1  # 0.08
        self.start_measure = False
        # ________________
        self.cmd_input = []
        self.Kp = [1, 1, 1]
        self.Ki = [0, 0, 0.0]
        self.Kd = [0.25, 0.25, 0.25]
        self.sample_time = 60
        self.prev_values = [0, 0, 0]
        self.max_values = 20
        self.min_values = 10
        self.error = [0, 0, 0]
        self.errsum=[0,0,0]

        self.Roll = 40
        self.Pitch = 40
        self.Yaw = 0
        self.Throttle = 30
        
        self.last_time = 0.0000
        self.now = 0.0000
        self.derr = [0,0,0]

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
    
    def get_current_state(self):
        return self.current_state

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
            print(f"cmd >> {cmd}")
            print(f"distance >> {distance}")

    def get_current_input(self):
        print(self.cmd_input)
        return self.cmd_input

    def pid(self,desired_state):
        self.desired_state = desired_state
        self.error[0] = self.current_state[0] - self.desired_state[0]
        self.error[1] = self.current_state[1] - self.desired_state[1]
        self.error[2] = self.current_state[2] - self.desired_state[2]
        self.now = int(round(time.time() * 1000))
        
        self.timechange = self.now - self.last_time

        if (self.timechange > self.sample_time):
            if (self.last_time != 0):
                # Integration for Ki
                self.errsum[0] = self.errsum[0]+(self.error[0]*self.timechange)
                self.errsum[1] = self.errsum[1]+(self.error[1]*self.timechange)
                self.errsum[2] = self.errsum[2]+(self.error[2]*self.timechange)
                # print(f"errorsum is {self.errsum}")
                # Derivation for Kd
                self.derr[0] = (
                    self.error[0]-self.prev_values[0])/self.timechange
                self.derr[1] = (
                    self.error[1]-self.prev_values[1])/self.timechange
                self.derr[2] = (
                    self.error[2]-self.prev_values[2])/self.timechange
                # print(f"derivatives are {self.derr}")
                # Calculating output in 30

                self.Roll = 30 - (self.Kp[0]*self.error[0])-(self.Kd[0]*self.derr[0])
                self.Pitch = 30 +  (self.Kp[1]*self.error[1])+(self.Kd[1]*self.derr[1])
                self.Throttle = 30 +  (self.Kp[2]*self.error[2])+(self.Kd[2] *
                                                self.derr[2])-(self.errsum[2]*self.Ki[2])

                # Checking min and max threshold and updating on true
                # Throttle Conditions
                if self.Throttle > 30:
                    self.Throttle = self.max_values
                if self.Throttle < -30:
                    self.Throttle = self.min_values

                # Pitch Conditions
                if self.Pitch > 30:
                    self.Pitch = self.max_values
                if self.Pitch < -30:
                    self.Pitch = self.min_values

                # Roll Conditions
                if self.Roll > 30:
                    self.Roll = self.max_values
                if self.Roll < -30:
                    self.Roll = self.min_values

                # Publishing values on topic 'drone command'
                self.cmd_input = [self.Roll,
                                  self.Pitch, 0, self.Throttle]

                # Updating prev values for all axis
                self.prev_values[0] = self.error[0]
                self.prev_values[1] = self.error[1]
                self.prev_values[2] = self.error[2]
             
            self.last_time = self.now

            return self.get_current_input(), self.error
           



if __name__ == "__main__":
    modelAgent = PIDcontroller("84:20:96:91:73:F1")
    modelAgent.start_and_prepare()
    waypoints =  [[0,0,0],[1,1,1]]
    
    # currenstate
    
    # calculate distance
    for desired_state in waypoints:
        distance = ((modelAgent.get_current_state()[0] - desired_state[0])**2 +
                    (modelAgent.get_current_state()[1] - desired_state[1])**2 +
                (modelAgent.get_current_state()[2] - desired_state[2])**2)**0.5
        while distance > 0.3:
            for i in range(size(waypoints, 0)):
                cmd, error = modelAgent.pid(waypoints[i])
                modelAgent.mambo.fly_direct(roll=cmd[0],
                                  pitch=cmd[1],
                                  yaw=cmd[2],
                                  vertical_movement=cmd[3],
                                  duration=None)
                modelAgent.mambo.smart_sleep(0.5)


    modelAgent.land_and_disconnect()
    

    # "84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone
