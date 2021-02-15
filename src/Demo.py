from src import ReflexAgent, ModelBasedAgent, KalmanFilter

# mambo = ModelBasedAgent("7A:64:62:66:4B:67")
# mambo = ReflexAgent("7A:64:62:66:4B:67")
# mambo = ReflexAgent("7A:64:62:66:4B:67")
# mambo = Drone("7A:64:62:66:4B:67")
# mambo = Drone("84:20:96:6c:22:67") #lab drone
mambo = ReflexAgent("84:20:96:6c:22:67") #lab drone
mambo.connected()
mambo.get_battery()

mambo.take_off()
# mambo.destination_sensor_based_kalman(1,0,0)
mambo.destination_sensor_based(1,0,0)
# mambo.square()
# mambo.test_shape()
# mambo.reset()
# mambo.forward_and_back()

# mambo.get_pos_xyz()

mambo.land()
mambo.get_pos_xyz()
# mambo.smart_sleep(3)
mambo.disconnect()
#first drone