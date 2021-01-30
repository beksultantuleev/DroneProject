from pyparrot.Minidrone import Mambo, Minidrone
import pyparrot

class Drone():
    def __init__(self, drone_id, drone_mac):
        self._drone_id = drone_id
        self._drone_mac = drone_mac

    def connect(self):
        self._drone_id = Mambo(self._drone_mac.islower(), use_wifi=True)
        success = self._drone_id.connect(num_retries=3)
        return success
    def disconnect(self):
        self._drone_id.disconnect()

    def get_connection_status(self):
        if self.connect():
            return "connected"
        return "not connected"
    
    def get_battery(self):
        self._drone_id.ask_for_state_update()
        return self._drone_id.sensors.battery
    
    def get_xyz(self):
        self._drone_id.ask_for_state_update()
        x = self._drone_id.sensors.quaternion_x
        y = self._drone_id.sensors.quaternion_y
        z = self._drone_id.sensors.quaternion_z
        return {"X":x, "Y":y, "Z":z}


home = Drone("first", "7A:64:62:66:4B:67")
# home.get_xyz()

if home.connect():
    home.get_battery()
# home.connect()
# home.get_battery()
# home.get_connection_status()
# home.get_xyz()
