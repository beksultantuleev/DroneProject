from MainDroneImproved import Drone

class DetectionDroneTest(Drone):
    def flight_func(self, args):
        if self.test_flying:
            print(f"battery level is {self.mambo.sensors.__dict__['battery']}")
            self.mambo.ask_for_state_update()
            # print(f"all dictionary is here >> {self.mambo.sensors.sensors_dict['DronePosition_posx']}")
            # print("Taking off!!")
            # self.mambo.safe_takeoff(5)

            if self.mambo.sensors.flying_state != "emergency":
                print("Sensor Calibration...")
                while self.mambo.sensors.speed_ts==0:
                    continue
                self.start_measure = True

                # print("Flying forward")
                # self.mambo.fly_direct(0,20,0,0,1)
                # self.mambo.smart_sleep(1)
                # print("going back")
                # self.mambo.fly_direct(0,-20,0,0,1)

                print("landing!!")
                self.mambo.safe_land(2)
            else:
                print("not gonna fly! smth wrong")
    
    def sensor_cb(self, args):
        print([
            self.mambo.sensors.speed_x,
            self.mambo.sensors.speed_y,
            self.mambo.sensors.speed_z
            # self.mambo.sensors.sensors_dict['DronePosition_posx']
        ])   

test_flying = True
drone_mac = "7A:64:62:66:4B:67" #first 
# drone_mac = "84:20:96:6c:22:67" #second
use_wifi = True

if __name__ == "__main__":
    detection_drone = DetectionDroneTest(test_flying, drone_mac, use_wifi)
    detection_drone.execute()