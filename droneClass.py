from pyparrot.Minidrone import Mambo
import pyparrot
import numpy as np


class Drone():
    def __init__(self, drone_mac):
        self._drone_id = None
        self._drone_mac = drone_mac

    def disconnect(self):
        print("disconnecting")
        self.smart_sleep()
        self._drone_id.disconnect()

    def connected(self):
        self._drone_id = Mambo(self._drone_mac.islower(), use_wifi=True)
        success = self._drone_id.connect(num_retries=3)
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
        self.ask_for_state_update()
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
        print(orientation)
        return orientation

    def take_off(self):
        self.smart_sleep(1)
        print("Safe take off!")
        self._drone_id.safe_takeoff(1)

    def land(self):
        print("Landing!")
        self._drone_id.safe_land(2)

    def fly_direct(self, roll, pitch, vertical_movement, duration):
        self._drone_id.fly_direct(roll=roll, pitch=pitch, yaw=0,
                                  vertical_movement=vertical_movement, duration=duration)

    def turn_right(self):
        self._drone_id.turn_degrees(90)

    def turn_left(self):
        self._drone_id.turn_degrees(-90)

    def turn_around(self):
        self._drone_id.turn_degrees(-180)

    def destination_no_sensor(self, x, y):
        if y > 0:
            while y != 0:
                self.fly_direct(0, 45, 0, 1)
                self.smart_sleep(1)
                y -= 1
        elif y < 0:
            while y != 0:
                self.fly_direct(0, -45, 0, 1)
                self.smart_sleep(1)
                y += 1
        self.smart_sleep()
        if x > 0:
            self.turn_right()
            while x != 0:
                self.fly_direct(0, 45, 0, 1)
                x -= 1
        elif x < 0:
            self.turn_left()
            while x != 0:
                self.fly_direct(0, 45, 0, 1)
                self.smart_sleep(1)
                x += 1

    def destination_sensor_based(self, x, y, z):
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
        Drone.__init__(self, drone_mac)
        '''safe landing during emergency'''
        def hover():
            if self.get_flying_state() == "hovering":
                self._drone_id.turn_degrees(90)
        self.hover = hover
        # def emergency():
        #     if self.get_flying_state() == "emergency":
        #         self.land()
        # self.emergency = emergency


class ModelBasedAgent(Drone):
    def __init__(self):
        Drone.__init__(self)

        def program():
            pass


class FlightPlanner(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)

    def destination_sensor_based_improved(self, x, y, z=None):
        # sensor scale in pos X  #move forward and backward, headlight of drone +
        # sensor scale in pos Y #move sideways, right is +
        # sensor in pos Z  # move up and down, move up is -
        pos_x_y_z = self.get_pos_xyz()
        # previous_state = None

        stop_value_x = int(x)
        stop_value_y = int(y)
        stop_value_z = int(-z)

        # get initial positions
        pos_x = np.round(float(pos_x_y_z["pos_X"]/100), 2)
        pos_y = np.round(int(pos_x_y_z["pos_Y"]/100), 2)
        pos_z = np.round(int(pos_x_y_z["pos_Z"]/100), 2)

        if x == y == z == 0:
            print("i am already here")
            return "I am already here"

        # move forward and backward
        if x > 0:
            while pos_x < stop_value_x:
                self.fly_direct(0, 45, 0, 1)
                self.smart_sleep(1)
                pos_x = np.round(int(self.get_pos_xyz()["pos_X"]/100),2)
        elif x < 0:
            self.turn_around()
            while pos_x > stop_value_x:
                self.fly_direct(0, 45, 0, 1)
                self.smart_sleep(1)
                pos_x = int(self.get_pos_xyz()["pos_X"]/100)
            self.turn_around()
        # move sideways
        if y > 0:
            self.turn_right()
            while pos_y < stop_value_y:
                self.fly_direct(0, 45, 0, 1)
                self.smart_sleep(1)
                pos_y = int(self.get_pos_xyz()["pos_Y"]/100)
            self.turn_left()
        elif y < 0:
            self.turn_left()
            while pos_y > stop_value_y:
                self.fly_direct(0, 45, 0, 1)
                self.smart_sleep(1)
                pos_y = int(self.get_pos_xyz()["pos_Y"]/100)
            self.turn_right()
        # move up and down
        if z > 0:
            while pos_z > stop_value_z:
                self.fly_direct(0, 0, 60, 1)
                self.smart_sleep(1)
                pos_z = int(self.get_pos_xyz()["pos_Z"]/100)
        elif z < 0:
            while pos_z < (-stop_value_z):
                self.fly_direct(0, 0, -50, 1)
                self.smart_sleep(1)
                pos_z = int(self.get_pos_xyz()["pos_Z"]/100)
        # previous_state = {}

    def square(self):
        self.destination_sensor_based_improved(1, -1, 0)
        self.destination_sensor_based_improved(-1, -1, 0)


mambo = FlightPlanner("7A:64:62:66:4B:67")
# mambo = ReflexAgent("7A:64:62:66:4B:67")
# mambo = Drone("7A:64:62:66:4B:67")
# mambo = Drone("84:20:96:6c:22:67") #lab drone
mambo.connected()
mambo.get_battery()

mambo.take_off()
# mambo.destination_sensor_based_improved(1,-1,0)
mambo.square()

# mambo.get_pos_xyz()

mambo.land()
mambo.get_pos_xyz()
# mambo.smart_sleep(3)
mambo.disconnect()
