from MainDrone import Drone
import numpy as np


class ReflexAgent(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)

    def setCoordinates(self, x, y, z=None):
        # sensor scale in pos X  #move forward and backward, headlight of drone +
        # sensor scale in pos Y #move sideways, right is +
        # sensor in pos Z  # move up and down, move up is -
        pos_x_y_z = self.get_pos_xyz()

        stop_value_x = (x)
        stop_value_y = (y)
        stop_value_z = (-z)

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


    def square(self):
        self.setCoordinates(1, -1, 0)
        # self.smart_sleep(1)
        self.setCoordinates(-1, -1, 0)
    
    def test_shape(self):
        self.setCoordinates(1, -1, 0)
        # self.smart_sleep()
        self.setCoordinates(2, 0, 0)
    
    def forward_and_back(self):
        self.setCoordinates(2,0,0)
        # self.smart_sleep(0.5)
        self.setCoordinates(-2,0,0)


if __name__ == "__main__":
    mambo = ReflexAgent("7A:64:62:66:4B:67")
    mambo.connected()
    mambo.get_battery()
    mambo.disconnect()