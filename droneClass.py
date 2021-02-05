from pyparrot.Minidrone import Mambo
import pyparrot
# import numpy as np
# from filterpy.kalman import KalmanFilter
# from filterpy.common import Q_discrete_white_noise



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

    def smart_sleep(self, time = None):
        if time is None:
            self._drone_id.smart_sleep(2)
        else:
            self._drone_id.smart_sleep(time)

    def request_all_sensor_data(self):
        self.smart_sleep(1)
        self._drone_id.ask_for_state_update()
        sensors = self._drone_id.sensors.__dict__
        self.smart_sleep(1)
        return sensors

    def get_all_sensor_data(self):
        self.smart_sleep(1)
        self._drone_id.ask_for_state_update()
        sensors = self._drone_id.sensors.__dict__
        self.smart_sleep(1)
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

    def get_altitude(self):
        sensors_dict = self.request_all_sensor_data()
        altitude = sensors_dict["altitude"]
        altitude_ts = sensors_dict["altitude_ts"]
        print(altitude, altitude_ts)
        return (altitude, altitude_ts)

    def ask_for_state_update(self):
        self._drone_id.ask_for_state_update()

    def get_pos_xyz(self):
        sensors_dict = self.request_all_sensor_data()
        x = sensors_dict["sensors_dict"]["DronePosition_posx"]
        y = sensors_dict["sensors_dict"]["DronePosition_posy"]
        z = sensors_dict["sensors_dict"]["DronePosition_posz"]
        orientation = {"pos_X": x, "pos_Y": y, "pos_Z": z}
        print(orientation)
        return orientation

    def take_off(self):
        # self.smart_sleep(1)
        print("Safe take off!")
        self._drone_id.safe_takeoff(2)

    def land(self):
        self.smart_sleep(1)
        print("Landing!")
        self._drone_id.safe_land(2)
        self.smart_sleep(1)

    def fly_direct(self, roll, pitch, vertical_movement, duration):
        self._drone_id.fly_direct(roll=roll, pitch=pitch, yaw=0,
                                  vertical_movement=vertical_movement, duration=duration)

    def turn_right(self):
        # self.smart_sleep()
        self._drone_id.turn_degrees(90)

    def turn_left(self):
        # self.smart_sleep()
        self._drone_id.turn_degrees(-90)
    
    def turn_around(self):
        # self.smart_sleep()
        self._drone_id.turn_degrees(180)

    def destination_no_sensor(self, x, y):
        if y > 0:
            while y!=0:  
                self.fly_direct(0, 50, 0, 1)
                y -= 1
        elif y < 0:
            while y != 0:
                # self.smart_sleep()
                self.fly_direct(0, -50, 0, 1)
                y+=1
        elif y == 0:
            pass
        # self.smart_sleep()
        if x > 0:
            self.turn_right()
            while x!=0:
                
                self.fly_direct(0, 50, 0, 1)
                x-=1
        elif x<0:
            self.turn_left()
            while x!=0:
                # self.smart_sleep()
                self.fly_direct(0, 50, 0, 1)
                x+=1
        elif x==0:
            pass
        # self.smart_sleep()

    def destination_sensor_based(self, x, y, z):
        one_meter_in_sensor = 150 #sensor scale
        one_meter_in_sensor_z = 100
        pos_x_y_z = self.get_pos_xyz()

        stop_value_y = y*one_meter_in_sensor
        stop_value_x = x*one_meter_in_sensor
        stop_value_z = (-z)*one_meter_in_sensor_z

        pos_x = int(pos_x_y_z["pos_X"])
        pos_y = int(pos_x_y_z["pos_Y"])
        pos_z = int(pos_x_y_z["pos_Z"])
        if y>0:
            while pos_x<stop_value_y:
                self.fly_direct(0,50,0,1.5)
                pos_x =int(self.get_pos_xyz()["pos_X"])
        elif y<0:
            while pos_x>stop_value_y:
                self.fly_direct(0,-50,0,1)
                pos_x = int(self.get_pos_xyz()["pos_X"])
        elif y==0:
            pass
        if x>0:
            while pos_y<stop_value_x:
                self.fly_direct(50,0,0,1)
                pos_y = int(self.get_pos_xyz()["pos_Y"])
        elif x<0:
            while pos_y>stop_value_x:
                self.fly_direct(-50,0,0,1)
                pos_y = int(self.get_pos_xyz()["pos_Y"])
        elif x ==0:
            pass
        
        if z>0:
            while pos_z>stop_value_z: #-50to -85 || -100
                self.fly_direct(0,0,50,1)
                pos_z = int(self.get_pos_xyz()["pos_Z"])
        elif z<0:
            while pos_z<(-stop_value_z): #0 to 50 || 100
                self.fly_direct(0,0,-50,1)
                pos_z = int(self.get_pos_xyz()["pos_Z"])
        elif z==0:
            pass

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
# class Flight

mambo = ReflexAgent("7A:64:62:66:4B:67")
# mambo = Drone("7A:64:62:66:4B:67")
# mambo = Drone("84:20:96:6c:22:67") #lab drone
mambo.connected()
mambo.get_battery()

mambo.take_off()
# mambo.fly_direct(0,50,0,1)
mambo.destination_sensor_based(0,0,0)

mambo.land()
mambo.smart_sleep(3)

mambo.disconnect()