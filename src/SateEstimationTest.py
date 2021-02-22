from Drone_main import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman

class StateEstimatioinTest(Drone):
    def __init__(self,drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        self.kalmanfilter = MamboKalman([0,0,0], [0,0,0])
        self.current_velocities = [] # for use with KalmanFilter (used as u (input))
        self.current_state = [] # meters
        self.desired_state = [1, 0, 1] # meters
        self.eps = 0.08
        self.start_measure = False

    def sensor_callback(self, args):
        if self.start_measure:
            self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]
            self.current_velocities = [self.mambo.sensors.speed_x,
                                    self.mambo.sensors.speed_y,
                                    self.mambo.sensors.speed_z]
            self.current_state = self.kalmanfilter.get_state_estimate(self.current_measurement,
                                                                        self.current_velocities)
            self.controller.set_current_state(self.current_state)


    def flight_function(self, args):
        """
        Takeoff, fly to (1, 0, 1) with units (m), land.
        """

        if self.mambo.sensors.flying_state != 'emergency':

            print('sensor calib:')
            while self.mambo.sensors.speed_ts == 0:
                continue
            self.start_measure = True

            print('getting first state')
            while self.current_state == []:
                continue

            self.controller.set_desired_state(self.desired_state)
            print('flying to position ', self.desired_state)
            while ( (self.current_state[0] - self.desired_state[0])**2 +
                    (self.current_state[1] - self.desired_state[1])**2 +
                    (self.current_state[2] - self.desired_state[2])**2 )**0.5 > self.eps:
                cmd = self.controller.calculate_cmd_input()
                print('current state:',self.current_state)
                print('cmd:          ',cmd)
                self.mambo.fly_direct(roll=cmd[0],
                                        pitch=cmd[1],
                                        yaw=cmd[2],
                                        vertical_movement=cmd[3],
                                        duration=None)
                self.mambo.smart_sleep(0.5)
        print('landing')
        self.mambo.safe_land(3)




if __name__ == "__main__":
    detection_drone = StateEstimatioinTest("84:20:96:91:73:F1")
    detection_drone.run()
     #"84:20:96:91:73:F1"<<new drone #"7A:64:62:66:4B:67" <<-Old drone