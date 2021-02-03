from pyparrot.Minidrone import Mambo
import pyparrot
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


class Drone():
    def __init__(self, drone_mac):
        self._drone_id = None
        self._drone_mac = drone_mac

    def disconnect(self):
        print("disconnecting")
        self._drone_id.disconnect()

    def connected(self):
        self._drone_id = Mambo(self._drone_mac.islower(), use_wifi=True)
        success = self._drone_id.connect(num_retries=3)
        print("Connection established")
        return success

    def request_all_sensor_data(self):
        self._drone_id.ask_for_state_update()
        sensors = self._drone_id.sensors.__dict__
        return sensors

    def get_all_sensor_data(self):
        self._drone_id.ask_for_state_update()
        sensors = self._drone_id.sensors.__dict__
        print(sensors)
        return sensors

    def get_flying_state(self):
        state = self.request_all_sensor_data()["flying_state"]
        print(state)
        return state

    def get_battery(self):
        battery = self.request_all_sensor_data()["battery"]
        print(battery)
        return battery

    def get_xyz(self):
        sensors_dict = self.request_all_sensor_data()
        x = sensors_dict["quaternion_x"]
        y = sensors_dict["quaternion_y"]
        z = sensors_dict["quaternion_z"]
        orientation = {"X": x, "Y": y, "Z": z}
        return orientation

    def get_pos_xyz(self):
        sensors_dict = self.request_all_sensor_data()
        x = sensors_dict["sensors_dict"]["DronePosition_posx"]
        y = sensors_dict["sensors_dict"]["DronePosition_posy"]
        z = sensors_dict["sensors_dict"]["DronePosition_posz"]
        orientation = [{"pos_X": x, "pos_Y": y, "pos_Z": z}]
        orientation.append(self.get_xyz())
        print(orientation)
        return orientation

    def take_off(self):
        self._drone_id.ask_for_state_update()
        self._drone_id.smart_sleep(2)
        print("Safe take off!")
        self._drone_id.safe_takeoff(3)

    def land(self):
        print("Landing!")
        self._drone_id.safe_land(5)
        self._drone_id.smart_sleep(5)

    def fly_direct(self, roll, pitch, yaw, vertical_movement, duration):
        self._drone_id.fly_direct(roll=roll, pitch=pitch, yaw=yaw,
                                  vertical_movement=vertical_movement, duration=duration)


class ReflexAgent(Drone):
    def __init__(self):
        Drone.__init__(self)
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
        # f = KalmanFilter(dim_x=2, dim_z=1)
        # f.x = np.array([2., 0.])
        # f.F = np.array([[1., 1.], [0., 1.]])
        # f.H = np.array([[1.,0.]])
        # f.P *= 1000
        # f.R = 5
        # f.Q = Q_discrete_white_noise(dim=2, dt= 0.1, var=0.13)
        # z = self.get_altitude()
        # f.predict()
        # f.update(z)

        def program():
            pass
# class Flight


mambo = Drone("7A:64:62:66:4B:67")
# mambo = Drone("84:20:96:6c:22:67") #lab drone
mambo.connected()
print("data collection begin!!")
mambo.get_battery()
mambo.get_pos_xyz()
mambo.take_off()
print(" take off data, look for Z!! its like base from air!!!")
mambo.get_pos_xyz()

print("moving forward (pitch 50)")
mambo.fly_direct(0, 50, 0, 0, 1)
print("end of movement")
mambo.get_pos_xyz()

mambo.land()
print("after landing")
mambo.get_pos_xyz()
mambo.disconnect()

# mambo = ReflexAgent("7A:64:62:66:4B:67")
# mambo.connected()
# mambo.get_altitude()
# mambo.get_battery()
# mambo.disconnect()
