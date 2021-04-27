import sys
import os
import inspect
import numpy as np

currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
from swarm.swarm_controllerUWB import SwarmLauncher
from ModelBasedAgentUWB import ModelBasedAgentUWB
# ________________________
swarm = SwarmLauncher()

# swarm.setWaypoints([1,0,1]) #to set manualy
# print(swarm.getWaypoints())

mambo1 = "D0:3A:49:F7:E6:22"
mambo2 = "D0:3A:0B:C5:E6:22" #new tag
mambo3 = "D0:3A:B1:DC:E6:20" #with no sticker

drone2 = ModelBasedAgentUWB(
    drone_mac=mambo2, use_wifi=False,
    controller=swarm.getController(), local=True,
    start_loggin=False, topic="Position3")
drone2.start_and_prepare()

for points in swarm.getWaypoints():
    drone2.go_to_xyz(points)
drone2.land_and_disconnect()
