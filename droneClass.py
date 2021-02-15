from pyparrot.Minidrone import Mambo
import pyparrot
import numpy as np


class Drone():
    def __init__(self, drone_mac):
        self._drone_id = None
        self._drone_mac = drone_mac

    def disconnect(self):
        print("disconnecting")
        self.smart_sleep(5) 
        self._drone_id.disconnect()

    def connected(self):
        self._drone_id = Mambo(self._drone_mac.islower(), use_wifi=True)
        success = self._drone_id.connect(num_retries=3)
        print("Sleeping")
        self.smart_sleep()
        self.ask_for_state_update()
        self.smart_sleep()
        print("Connection established")
        return success

    def smart_sleep(self, time=None):
        if time is None:
            self._drone_id.smart_sleep(2)
        else:
            self._drone_id.smart_sleep(time)

    def print_all_sensor_data(self):
        # self._drone_id.ask_for_state_update() #try without it
        sensors = self._drone_id.sensors.__dict__
        print(sensors)

    def get_all_sensor_data(self):
        # self._drone_id.ask_for_state_update()
        sensors = self._drone_id.sensors.__dict__
        return sensors

    def get_flying_state(self):
        state = self.get_all_sensor_data()["flying_state"]
        print(state)
        return state

    def get_battery(self):
        # self.ask_for_state_update()
        battery = self.get_all_sensor_data()["battery"]
        print(battery)
        return battery

    def get_altitude(self):
        sensors_dict = self.get_all_sensor_data()
        altitude = sensors_dict["altitude"]
        altitude_ts = sensors_dict["altitude_ts"]
        print(altitude, altitude_ts)
        return (altitude, altitude_ts)

    def ask_for_state_update(self):
        self._drone_id.ask_for_state_update()

    def get_pos_xyz(self):
        sensors_dict = self.get_all_sensor_data()
        x = sensors_dict["sensors_dict"]["DronePosition_posx"]
        y = sensors_dict["sensors_dict"]["DronePosition_posy"]
        z = sensors_dict["sensors_dict"]["DronePosition_posz"]
        orientation = {"pos_X": x, "pos_Y": y, "pos_Z": z}
        try:
            print( orientation)
        except:
            print("no data")
        return orientation

    def take_off(self):
        self.smart_sleep(1)
        print("Safe take off!")
        self._drone_id.safe_takeoff(1)

    def land(self):
        print("Landing!")
        self._drone_id.safe_land(5) #5 in tutorial, but i put 2?

    def fly_direct(self, roll, pitch, vertical_movement, duration):
        self._drone_id.fly_direct(roll=roll, pitch=pitch, yaw=0,
                                  vertical_movement=vertical_movement, duration=duration)

    def fly_direct_fixed(self):
        #this function is adjustable to fly straight
        self.fly_direct(0,5,0,0.5) #try 40 and 1.3

    def turn_right(self):
        self._drone_id.turn_degrees(90)
        self.smart_sleep(1.2)

    def turn_left(self):
        self._drone_id.turn_degrees(-90)
        self.smart_sleep(1.2)

    def turn_around(self):
        self._drone_id.turn_degrees(-180)
        self.smart_sleep(1.2)

    
    def destination(self, x, y, z):
        # sensor scale in pos X aka y in cartesian plane #move forward and backward
        one_meter_in_sensor_x = 80
        # sensor scale in pos Y aka x in cartesian plane #move sideways
        one_meter_in_sensor_y = 80
        one_meter_in_sensor_z = 90  # move up and down
        pos_x_y_z = self.get_pos_xyz()

        stop_value_x = x*one_meter_in_sensor_y
        stop_value_y = y*one_meter_in_sensor_x
        stop_value_z = (-z)*one_meter_in_sensor_z

        pos_x = int(pos_x_y_z["pos_X"])
        pos_y = int(pos_x_y_z["pos_Y"])
        pos_z = int(pos_x_y_z["pos_Z"])
        if x == y == z == 0:
            print("not gonna fly")
            return "not gonna fly"

        # move forward and backward
        if y > 0:
            while pos_x < stop_value_y:
                self.fly_direct(0, 45, 0, 1)
                self.smart_sleep(0.5)
                pos_x = int(self.get_pos_xyz()["pos_X"])

        elif y < 0:
            while pos_x > stop_value_y:
                self.fly_direct(0, -45, 0, 1)
                self.smart_sleep(0.5)
                pos_x = int(self.get_pos_xyz()["pos_X"])
        # self.smart_sleep(1)

        # move sideway
        if x > 0:
            while pos_y < stop_value_x:
                self.fly_direct(45, 0, 0, 1)
                self.smart_sleep(0.5)
                pos_y = int(self.get_pos_xyz()["pos_Y"])
        elif x < 0:
            while pos_y > stop_value_x:
                self.fly_direct(-45, 0, 0, 1)
                self.smart_sleep(0.5)
                pos_y = int(self.get_pos_xyz()["pos_Y"])
        # self.smart_sleep(1)

        # moves up and down
        if z > 0:
            while pos_z > stop_value_z:  # -50to -85 || -100
                self.fly_direct(0, 0, 50, 1)
                self.smart_sleep(0.5)
                pos_z = int(self.get_pos_xyz()["pos_Z"])
        elif z < 0:
            while pos_z < (-stop_value_z):  # 0 to 50 || 100
                self.fly_direct(0, 0, -50, 1)
                self.smart_sleep(0.5)
                pos_z = int(self.get_pos_xyz()["pos_Z"])



class ReflexAgent(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)

    def destination_sensor_based(self, x, y, z=None):
        # sensor scale in pos X  #move forward and backward, headlight of drone +
        # sensor scale in pos Y #move sideways, right is +
        # sensor in pos Z  # move up and down, move up is -
        pos_x_y_z = self.get_pos_xyz()
        coefficient = 1 #adjusting variable

        stop_value_x = (x) * coefficient
        stop_value_y = (y) * coefficient
        stop_value_z = (-z) * coefficient

        # get initial positions
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
                # self.smart_sleep(0.01)
                pos_x = np.round((self.get_pos_xyz()["pos_X"]/100), 2)
                print( pos_x)
            print("end while")
            self.fly_direct(0,-45,0,0.1)
            print("negatve")
        elif x < 0:
                self.fly_direct(0,-45,0,1.1)
                self.smart_sleep(1)
                pos_x = np.round((self.get_pos_xyz()["pos_X"]/100), 2)
            # self.turn_around()
        # move sideways
        if y > 0:
            self.turn_right()
            while pos_y < stop_value_y:
                self.fly_direct_fixed()
                self.smart_sleep(1)
                pos_y = np.round((self.get_pos_xyz()["pos_Y"]/100), 2)
            self.turn_left()
        elif y < 0:
            self.turn_left()
            while pos_y > stop_value_y:
                self.fly_direct_fixed()
                self.smart_sleep(1)
                pos_y = np.round((self.get_pos_xyz()["pos_Y"]/100), 2)
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

    def reset(self):
        self.destination_sensor_based(0,0,0)

    def square(self):
        self.destination_sensor_based(1, -1, 0)
        # self.smart_sleep(1)
        self.destination_sensor_based(-1, -1, 0)
    
    def test_shape(self):
        self.destination_sensor_based(1, -1, 0)
        # self.smart_sleep()
        self.destination_sensor_based(2, 0, 0)
    
    def forward_and_back(self):
        self.destination_sensor_based(2,0,0)
        # self.smart_sleep(0.5)
        self.destination_sensor_based(-2,0,0)


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

# mambo = ModelBasedAgent("7A:64:62:66:4B:67")
# mambo = ReflexAgent("7A:64:62:66:4B:67")
# mambo = ReflexAgent("7A:64:62:66:4B:67")
# mambo = Drone("7A:64:62:66:4B:67")
# mambo = Drone("84:20:96:6c:22:67") #lab drone
mambo = ReflexAgent("84:20:96:6c:22:67") #lab drone
mambo.connected()
mambo.get_battery()

mambo.take_off()
# mambo.destination_sensor_based_kalman(1,0,0)
mambo.destination_sensor_based(1,0,0)
# mambo.square()
# mambo.test_shape()
# mambo.reset()
# mambo.forward_and_back()

# mambo.get_pos_xyz()

mambo.land()
mambo.get_pos_xyz()
# mambo.smart_sleep(3)
mambo.disconnect()
#first drone
