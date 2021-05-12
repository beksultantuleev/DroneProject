import subprocess


class SwarmLauncher():
    def __init__(self):
        self.finalBashCommand = ""
        self.location = "python src/swarm/"
        # square = [[0.5, 0, 1], [0.5, -0.5, 1], [0.5, -0.5, 1], [0.5, 0.5, 1]]
        self.waypoints = [[1, 0, 1]]
        self.controller = "lqr"
        self.local = True

    def addUav(self, filename):
        self.finalBashCommand += self.location + filename + " & "


    def getWaypoints(self):
        return self.waypoints

    def getCoordinateSystem(self):
        return self.local

    def getController(self):
        return self.controller

    def getFinalBashCommand(self):
        print(self.finalBashCommand[:-2])
    
    def setWaypoints(self, list_of_waypoints):
        "this shold be list of lists"
        self.waypoints = list_of_waypoints

    def launch(self):
        subprocess.call(
            self.finalBashCommand[:-2], shell=True)


if __name__ == "__main__":
    swarm = SwarmLauncher()
    swarm.addUav("UWBmambo1.py")
    swarm.addUav("UWBmambo2.py")

    # swarm.getFinalBashCommand()
    swarm.launch()
