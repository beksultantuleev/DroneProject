# from test_src.Drone_main import Drone
from Drone_main import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman

class ReflexAgent(Drone):

    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.controller = MamboPositionController()
        self.kalmanfilter = MamboKalman([0, 0, 1], [0, 0, 0]) #pos and vel
        self.current_measurement = []
        self.current_state = []
    
    def sensor_callback(self, args):
        self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,self.mambo.sensors.sensors_dict['DronePosition_posy']/100,self.mambo.sensors.sensors_dict['DronePosition_posz']/-100 ]
        self.current_velocity = [self.mambo.sensors.speed_x,self.mambo.sensors.speed_y, self.mambo.sensors.speed_z]
        self.current_state = self.kalmanfilter.get_state_estimate(self.current_measurement, self.current_velocity)
        print(f"after kalman filter>> {self.current_state}")
        print(f"before kalman filter >> {self.current_measurement}")

    def flight_function(self, args):

        if self.mambo.sensors.flying_state != "emergency":
            print("Sensor Calibration...")
            while self.mambo.sensors.speed_ts==0:
                continue
            self.mambo.fly_direct(0,30,0,0,1)
        else:
            print("not gonna fly")
            
if __name__ == "__main__":
    detection_drone = ReflexAgent("7A:64:62:66:4B:67")
    detection_drone.run()

    
    