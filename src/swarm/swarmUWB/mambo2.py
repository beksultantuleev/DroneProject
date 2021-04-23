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

mambo1 = "D0:3A:49:F7:E6:22"
mambo2 = "D0:3A:0B:C5:E6:22"

drone2 = ModelBasedAgentUWB(
    drone_mac=mambo2, use_wifi=False,
    controller=swarm.getController(), local=True,
    start_loggin=False, topic="Position1")
drone2.start_and_prepare()

for points in swarm.getWaypoints():
    drone2.go_to_xyz(points)
drone2.land_and_disconnect()
