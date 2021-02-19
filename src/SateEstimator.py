from Drone_main import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman


class StateEstimator(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        self.kalmanfilter = MamboKalman([0, 0, 1], [0, 0, 0]) #pos and vel
        self.current_velocity = []
        self.current_state = []
        self.desired_state = []
        self.eps = 0.08
        self.max_alt = 2
        self.start_measure = False #not implemented yet

    def sensor_callback(self, args):
        self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,self.mambo.sensors.sensors_dict['DronePosition_posy']/100,self.mambo.sensors.sensors_dict['DronePosition_posz']/-100 ]
        self.current_velocity = [self.mambo.sensors.speed_x,self.mambo.sensors.speed_y, self.mambo.sensors.speed_z]
        self.current_state = self.kalmanfilter.get_state_estimate(self.current_measurement, self.current_velocity)
        self.controller.set_current_state(self.current_state)
        # print(f"the current state is {self.current_state[0]}")
        
        
    
    def go_to_xyz(self, desired_state):
        self.desired_state = desired_state
        self.controller.set_desired_state(self.desired_state)

        dist = ((self.current_state[0] - self.desired_state[0])**2 +
                (self.current_state[1] - self.desired_state[1])**2 +
                (self.current_state[2] - self.desired_state[2])**2)**0.5
        # print(dist)
        while dist> self.eps:
            cmd = self.controller.calculate_cmd_input()
            self.mambo.fly_direct(roll=cmd[1],
                                  pitch=cmd[0],
                                  yaw=cmd[2],
                                  vertical_movement=cmd[3],
                                  duration=None)
            print(dist)
            dist = ((self.current_state[0] - self.desired_state[0])**2 +
                (self.current_state[1] - self.desired_state[1])**2 +
                (self.current_state[2] - self.desired_state[2])**2)**0.5
        
    def flight_function(self, args):

        if self.mambo.sensors.flying_state != "emergency":
            print("Sensor Calibration...")
            while self.mambo.sensors.speed_ts==0:
                continue
            # self.start_measure = True
            # print("moving")
            # self.mambo.fly_direct(0,30,0,0,3)
            # print("all sensors")
            # print(self.mambo.sensors.sensors_dict)

            # print("Flying forward")
            self.mambo.fly_direct(0,1,0,0,0.5) #you need to start flying to avoid error
            print("my boy here")
            self.go_to_xyz([-1,0,1])

            # self.mambo.smart_sleep(1)
            # print("going back")
            # self.mambo.fly_direct(0,-20,0,0,1)

        else:
            print("not gonna fly! smth wrong")
    
    

if __name__ == "__main__":
    detection_drone = StateEstimator("7A:64:62:66:4B:67")
    detection_drone.run()
    
 
