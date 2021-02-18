# from test_src.Drone_main import Drone
from Drone_main import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman


class DetectionDroneTest(Drone):
    def flight_function(self, args):
        self.controller = MamboPositionController()
        self.kalmanfilter = MamboKalman([0, 0, 1], [0, 0, 0])

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
            # self.mambo.fly_direct(0,50,0,0,1)


        else:
            print("not gonna fly! smth wrong")
    
    def sensor_callback(self, args):
        print([self.mambo.sensors.sensors_dict['DronePosition_posx']])

if __name__ == "__main__":
    detection_drone = DetectionDroneTest("7A:64:62:66:4B:67")
    detection_drone.run()
    
 
