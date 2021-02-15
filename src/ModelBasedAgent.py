from src import ReflexAgent
import numpy as np

class ModelBasedAgent(ReflexAgent):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)

    def kalman_filter(self, x, P, measurement, R, u, Q, F, H):
        """
            Implements a basic Kalman Filter Algorithm algorithm
            Input:
            x - initial state, [x1, x2, x0_dot, x1_dot]
            P - Covariance matrix, initial uncertainty
            measurement, observed position
            R - Measurement Noise/Uncertainty.
            u - external motion
            Q - Motion Noise
            F - Next State Function
            H - Measurement Function
            """
        # Update:
        y = np.matrix(measurement).transpose() - H * x
        S = H * P * H.transpose() + R  # residual convariance
        K_t = P * H.transpose() * S.I  # Kalman gain
        x = x + K_t*y  #state update estimate
        I = np.matrix(np.eye(F.shape[0])) # identity matrix
        P = (I - K_t*H)*P

        # Predict:
        x = F*x + u
        P = F*P*F.transpose() + Q

        return x, P

    def kalman_applyer(self):
        x = np.matrix([0, 0, 0, 0]).transpose()  # Initial state, at (0,0), at rest.
        P = np.matrix(np.eye(4))*1000  # initial uncertainty
        F = np.matrix([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])  #next state function
        H = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0]]) # Measurement function
        u = np.matrix([[0, 0, 0, 0]]).transpose() # external motion
        Q = np.eye(4) # motion noise
        R = 0.01 ** 2 #measurement noise, unseartinty

        pos_x_data = []
        pos_y_data = []
        result = []
        pos_x_data.append(np.round((self.get_pos_xyz()["pos_X"]/100), 2))
        pos_y_data.append(np.round((self.get_pos_xyz()["pos_Y"]/100), 2))
        for measurements in zip(pos_x_data, pos_y_data):
            x, P = self.kalman_filter(x, P, measurements, R, u, Q, F, H)
            result.append((x[:2]).tolist())
        kalman_x, kalman_y = zip(*result)
        return result

    def destination_sensor_based_kalman(self, x, y, z):

        x = np.matrix([0, 0, 0, 0]).transpose()  # Initial state, at (0,0), at rest.
        P = np.matrix(np.eye(4))*1000  # initial uncertainty
        F = np.matrix([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])  #next state function
        H = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0]]) # Measurement function
        u = np.matrix([[0, 0, 0, 0]]).transpose() # external motion
        Q = np.eye(4) # motion noise
        R = 0.01 ** 2 #measurement noise, unseartinty

        pos_x_y_z = self.get_pos_xyz()
        coefficient = 1 #adjusting variable

        stop_value_x = (x) * coefficient
        stop_value_y = (y) * coefficient
        stop_value_z = (-z) * coefficient

        # get initial positions 0, 0, 0
        pos_x = np.round((pos_x_y_z["pos_X"]/100), 2)
        pos_y = np.round((pos_x_y_z["pos_Y"]/100), 2)
        pos_z = np.round((pos_x_y_z["pos_Z"]/100), 2)

        if x == y == z == 0:
            print("i am already here")
            return "I am already here"

        # move forward and backward
        if x > 0:
            while pos_x < stop_value_x:
                self.fly_direct_fixed()
                self.smart_sleep(1)
                pos_x = self.kalman_applyer()[-1][0]
        elif x < 0:
            # self.turn_around()
            while pos_x > stop_value_x:
                # self.fly_direct_fixed()
                self.fly_direct(0,-45,0,1.1)
                self.smart_sleep(1)
                pos_x = self.kalman_applyer()[-1][0]
            # self.turn_around()
        # move sideways
        if y > 0:
            self.turn_right()
            while pos_y < stop_value_y:
                self.fly_direct_fixed()
                self.smart_sleep(1)
                pos_x = self.kalman_applyer()[-1][1]
            self.turn_left()
        elif y < 0:
            self.turn_left()
            while pos_y > stop_value_y:
                self.fly_direct_fixed()
                self.smart_sleep(1)
                pos_x = self.kalman_applyer()[-1][1]
            self.turn_right()
        # move up and down
        if z > 0:
            while pos_z > stop_value_z:
                self.fly_direct(0, 0, 50, 1)
                self.smart_sleep(0.2)
                pos_z = np.round((self.get_pos_xyz()["pos_Z"]/100), 2)
        elif z < 0:
            while pos_z < (-stop_value_z):
                self.fly_direct(0, 0, -50, 1)
                self.smart_sleep(0.2)
                pos_z = np.round((self.get_pos_xyz()["pos_Z"]/100), 2)
