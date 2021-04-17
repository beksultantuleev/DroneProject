from ModelBasedAgent import ModelBasedAgent
import threading
import time

mambo1 = "D0:3A:49:F7:E6:22"
mambo2 = "D0:3A:0B:C5:E6:22"

drone2 = ModelBasedAgent(mambo2, False, "lqr")
drone2.start_and_prepare()
# square = [[0.5, 0, 1], [0.5, -0.5, 1], [0.5, -0.5, 1], [0.5, 0.5, 1]]
square = [[1, 0, 1]]
for points in square:
    drone2.go_to_xyz(points)
drone2.land_and_disconnect()