import sys
import os
import inspect
# import numpy as np

currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
# ________________________
from ModelBasedAgentUWB import ModelBasedAgentUWB
from swarm.swarm_controllerUWB import SwarmLauncher
swarm = SwarmLauncher()

# swarm.setWaypoints([1,0,1]) #to set manualy 
# print(swarm.getWaypoints())
mambo1 = "D0:3A:49:F7:E6:22"
mambo2 = "D0:3A:0B:C5:E6:22" #new tag
mambo3 = "D0:3A:B1:DC:E6:20" #with no sticker


drone1 = ModelBasedAgentUWB(
    drone_mac=mambo1, use_wifi=False,
    controller=swarm.getController(), 
    local=swarm.getCoordinateSystem(),
    topic="Position1",
    start_loggin=False)
drone1.start_and_prepare()

for points in swarm.getWaypoints():
    drone1.go_to_xyz(points)
drone1.land_and_disconnect()
