from Drone_main import Drone

class DetectionDroneTest(Drone):
    def flight_func(self, args):
        

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
    
 
