
from pyparrot.Minidrone import Mambo

import json
import time

# If you are using BLE: you will need to change this to the address of YOUR mambo
# if you are using Wifi, this can be ignored
mamboAddr = "84:20:96:6c:22:67"

# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
mambo = Mambo(mamboAddr, use_wifi=True)

print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)


counter = 1

def default(x):
    return str(x)

while counter < 5:

    try:
        # mambo.sensors.update()
        mambo.ask_for_state_update()
        sensors = mambo.sensors.__dict__
        print(json.dumps(sensors, default=default,  indent=1))
        time.sleep(0.5)
    except KeyboardInterrupt:
        mambo.disconnect()
        exit()