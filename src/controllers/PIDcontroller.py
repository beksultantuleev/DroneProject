from numpy.core.fromnumeric import size

import numpy as np
import time


class PIDcontroller:
    def __init__(self):
        self.current_state = []  # meters
        self.desired_state = []  # meters #setpoint
        # ________________
        self.cmd_input = []
        
        self.Kp = np.ones((1, 3))[0] * 7    # [7,7,7]
        self.Ki =np.ones((1, 3))[0] * 0.2  # [0.2, 0.2, 0.2]
        self.Kd =np.ones((1, 3))[0] * 0.25  # [0.25, 0.25, 0.25]
        self.sample_time = 60
        self.prev_error_values = [0, 0, 0]
        self.max_values = 30
        self.min_values = -30
        self.error = [0, 0, 0]
        self.errsum = [0, 0, 0]

        self.last_time = 0.0001

        self.derr = [0, 0, 0]

    def set_desired_state(self, desired_state):
        self.desired_state = desired_state

    def set_current_state(self, current_state):
        self.current_state = current_state

    def calculate_cmd_input(self):
        self.pid(self.desired_state)
        # return self.cmd_input
        return self.adjusted_cmd

    def pid(self, desired_state):
        self.desired_state = desired_state
        self.error = list(np.subtract(self.current_state, self.desired_state))
        # print(f"self erros is >>{self.error}")
        self.now = int(round(time.time() * 1000))

        self.timechange = self.now - self.last_time

        if (self.timechange > self.sample_time):
            # print(self.sample_time)
            if (self.last_time != 0):
                # Integration for Ki
                if self.timechange >1000:
                    self.timechange =100
                    self.errsum = list(np.array(self.errsum) +np.array(self.error)* self.timechange/100)
                # print(f"errorsum is {self.errsum}")
                # print(f'error is >{self.error} and timechange is> {self.timechange} errsum is >{self.errsum}')
                # Derivation for Kd

                self.derr = list((np.array(self.error)- np.array(self.prev_error_values))/self.timechange)
                # print(f"derivatives are {self.derr[1]}")
                # Calculating output in 30

                self.Pitch =-(self.Kp[0]*self.error[0]) - \
                    (self.Kd[0]*self.derr[0]) -(self.errsum[0]*self.Ki[0]) 
                # print((self.errsum[0]*self.Ki[0]))
                # print(f'errorsum is >>{self.errsum} and Ki is >>{self.Ki}, product is {self.errsum[0]*self.Ki[0]}')
                self.Roll =-(self.Kp[1]*self.error[1]) + \
                    (self.Kd[1]*self.derr[1]) -(self.errsum[1]*self.Ki[1])
                self.Throttle = -(self.Kp[2]*self.error[2]) + \
                    (self.Kd[2]*self.derr[2])-(self.errsum[2]*self.Ki[2])
                # Checking min and max threshold and updating on true
                # Throttle Conditions
                if self.Throttle > self.max_values:
                    self.Throttle = self.max_values
                if self.Throttle < self.min_values:
                    self.Throttle = self.min_values

                # Pitch Conditions
                if self.Pitch > self.max_values:
                    self.Pitch = self.max_values
                if self.Pitch < self.min_values:
                    self.Pitch = self.min_values

                # Roll Conditions
                if self.Roll > self.max_values:
                    self.Roll = self.max_values
                if self.Roll < self.min_values:
                    self.Roll = self.min_values

                # Publishing values on topic 'drone command'
                self.cmd_input = [self.Roll,
                                  self.Pitch, 0, self.Throttle]
                self.adjusted_cmd = []
                for i in self.cmd_input:
                    if i>0:
                        if abs(i) < 0.1:
                            i = 0
                      
                        elif (abs(i) >= 0.1 and abs(i) <= 5):
                            i = 5
                        self.adjusted_cmd.append(i)                    
                    else:
                        if abs(i) < 0.1:
                            i = 0
                        elif (abs(i) >= 0.1 and abs(i) <= 5):
                            i = -5
                        self.adjusted_cmd.append(i)
                # print(self.adjusted_cmd)
                # Updating prev values for all axis
                self.prev_error_values = self.error

            self.last_time = self.now

            # return self.get_current_input(), self.error


if __name__ == "__main__":
    mambo = PIDcontroller()
    # ====================
    # mambo.set_current_state([0, 0, 0])
    # mambo.set_desired_state([1, 0, 0])
    # u = mambo.calculate_cmd_input()
    # print(u)
    # ===================
    destX = -10
    num = 0
    mambo.set_desired_state([destX, 0, 0])
    while num >destX:

        mambo.set_current_state([num,0,0])
        u = mambo.calculate_cmd_input()
        num -=0.5
        time.sleep(0.1)
        print(f"{u} at position>> {num}")
# ========================================
    # destY = 3
    # num = 0
    # mambo.set_desired_state([0, destY, 0])
    # while num <destY:

    #     mambo.set_current_state([0,num,0])
    #     u = mambo.calculate_cmd_input()
    #     num +=0.5
    #     time.sleep(0.1)
    #     print(f"{u} at position>> {num}")
# ========================================
    # destZ = 3
    # num = 0
    # mambo.set_desired_state([0, 0, destZ])
    # while num <destZ:

    #     mambo.set_current_state([0,0,num])
    #     u = mambo.calculate_cmd_input()
    #     num +=0.5
    #     time.sleep(0.1)
    #     print(f"{u} at position>> {num}")
