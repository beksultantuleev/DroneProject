from Drone_main import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman


class DetectionDroneTest(Drone):
    def __init__(self, drone_mac):
        super().__init__(drone_mac)
        self.start_measure = False

    def flight_function(self, args):
        if self.mambo.sensors.flying_state != "emergency":
            print("Sensor Calibration...")
            while self.mambo.sensors.speed_ts==0:
                continue
            self.start_measure = True
            self.mambo.fly_direct(0,30,0,0,1)


        else:
            print("not gonna fly! smth wrong")
    
    def sensor_callback(self, args):
        if self.start_measure:
            self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]
        
if __name__ == "__main__":
    detection_drone = DetectionDroneTest("7A:64:62:66:4B:67")
    detection_drone.run()
    
 
