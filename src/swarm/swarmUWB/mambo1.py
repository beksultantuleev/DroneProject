from swarm.swarmUWB.swarm_controllerUWB import SwarmLauncher
from ModelBasedAgentUWB import ModelBasedAgentUWB
import sys
import os
import inspect
import numpy as np

currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
# ________________________
swarm = SwarmLauncher()

# swarm.setWaypoints([1,0,1]) #to set manualy 

mambo1 = "D0:3A:49:F7:E6:22"
mambo2 = "D0:3A:0B:C5:E6:22"

drone1 = ModelBasedAgentUWB(
    drone_mac=mambo1, use_wifi=False,
    controller=swarm.getController(), local=swarm.getCoordinateSystem(),
    start_loggin=False, topic="Position1")
drone1.start_and_prepare()

for points in swarm.getWaypoints():
    drone1.go_to_xyz(points)
drone1.land_and_disconnect()
