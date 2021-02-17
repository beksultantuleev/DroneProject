from ReflexAgent import ReflexAgent
from MainDrone import Drone

# mambo = ModelBasedAgent("7A:64:62:66:4B:67")
mambo = ReflexAgent("7A:64:62:66:4B:67")
# mambo = ReflexAgent("7A:64:62:66:4B:67")
# mambo = Drone("7A:64:62:66:4B:67")
# mambo = Drone("84:20:96:6c:22:67") #lab drone
# mambo = ReflexAgent("84:20:96:6c:22:67") #lab drone
mambo.connected()
mambo.get_battery()

mambo.take_off()
mambo.fly_direct(0,50,0,None)
# mambo.setCoordinates()

mambo.land()
# mambo.get_pos_xyz()
# mambo.smart_sleep(3)
mambo.disconnect()
#first drone