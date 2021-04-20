import subprocess

subprocess.call(
    "python src/swarm/mambo1.py & python src/swarm/mambo2.py", shell=True)
