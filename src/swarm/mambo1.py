import sys
import os
import inspect

currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
#________________________
from ModelBasedAgent import ModelBasedAgent
from swarm.swarm_controller import SwarmLauncher
swarm = SwarmLauncher()

mambo1 = "D0:3A:49:F7:E6:22"
mambo2 = "D0:3A:0B:C5:E6:22"

drone1 = ModelBasedAgent(mambo1, False, swarm.getController(), False)
drone1.start_and_prepare()

for points in swarm.getWaypoints():
    drone1.go_to_xyz(points)
drone1.land_and_disconnect()


