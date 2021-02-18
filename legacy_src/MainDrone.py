from pyparrot.Minidrone import Mambo
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
        # self.fly_direct(0,5,0,0.5) #try 40 and 1.3
        pass

    def turn_right(self):
        self._drone_id.turn_degrees(90)
        self.smart_sleep(1.2)

    def turn_left(self):
        self._drone_id.turn_degrees(-90)
        self.smart_sleep(1.2)

    def turn_around(self):
        self._drone_id.turn_degrees(-180)
        self.smart_sleep(1.2)

if __name__ == "__main__":
    mambo = Drone("7A:64:62:66:4B:67")
    mambo.connected()
    mambo.get_battery()
    mambo.disconnect()