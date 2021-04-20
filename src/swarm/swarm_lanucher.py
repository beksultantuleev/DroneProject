import subprocess

class SwarmLauncher():
    def __init__(self):
        self.finalBashCommand = ""
        self.location = "python src/swarm/"

    def addUav(self, filename):
        self.finalBashCommand += self.location + filename + " & "

    def getFinalBashCommand(self):
        print(self.finalBashCommand[:-2])

    def launch(self):
        subprocess.call(
            self.finalBashCommand[:-2], shell=True)


if __name__ == "__main__":
    test = SwarmLauncher()
    test.addUav("mambo1.py")
    test.addUav("mambo2.py")
    # test.getFinalBashCommand()
    test.launch()


# subprocess.call(
#     "python src/swarm/mambo1.py & python src/swarm/mambo2.py", shell=True)
