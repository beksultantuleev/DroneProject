from pyparrot.Minidrone import Mambo

mamboMac = "7A:64:62:66:4B:67"
mambo = Mambo(mamboMac.islower(), use_wifi=True)
print("trying to connect")

success = mambo.connect(num_retries=3)
print(f""" connected {success}""")

if success:
    print("sleeping")
    mambo.smart_sleep(2)
    mambo.ask_for_state_update()
    mambo.smart_sleep(2)


    print("taking off!")
    mambo.safe_takeoff(5)


    print("landing")
    mambo.safe_land(5)


    mambo.smart_sleep(5)

    print("disconnect")
    mambo.disconnect()