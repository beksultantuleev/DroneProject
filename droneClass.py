from pyparrot.Minidrone import Mambo, Minidrone
import pyparrot

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
    
    def get_flying_state(self):
        if self.connected():
            state = self._drone_id.sensors.flying_state
            print(state)
            return state
    
    def get_battery(self):
        if self.connected():
            self._drone_id.ask_for_state_update()
            battery = self._drone_id.sensors.battery
            print(battery)
            return battery      
    
    def get_xyz(self):
        if self.connected():
            self._drone_id.ask_for_state_update()
            x = self._drone_id.sensors.quaternion_x
            y = self._drone_id.sensors.quaternion_y
            z = self._drone_id.sensors.quaternion_z
            orientation = {"X":x, "Y":y, "Z":z}
            print(orientation)
            return orientation
    
    def get_altitude(self):
        if self.connected():
            self._drone_id.ask_for_state_update()
            altitude_ts = self._drone_id.sensors.altitude_ts
            print(altitude_ts)
            return altitude_ts

    
    def take_off(self):
        if self.connected():
            self._drone_id.smart_sleep(2)
            print("Safe take off!")
            self._drone_id.safe_takeoff(5)
            self._drone_id.smart_sleep(3)
    
    def land(self):
        if self.connected():
            print("Landing!")
            self._drone_id.safe_land(5)
            self._drone_id.smart_sleep(5)
            self.disconnect()


class ReflexAgent:
    def __init__(self):
        pass

mambo = Drone("7A:64:62:66:4B:67")
# mambo.connected()
# mambo.get_battery()
# mambo.get_flying_state()
# mambo.get_xyz()

mambo.get_altitude()
# mambo.get_battery()
mambo.disconnect()
