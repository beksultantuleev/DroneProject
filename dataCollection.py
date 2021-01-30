from pyparrot.Minidrone import Mambo

mamboMac = "7A:64:62:66:4B:67"
mambo_bluetooth_adress = "80:19:34:EB:35:56" 
mambo = Mambo(mamboMac.islower(), use_wifi=True)
# mambo = Mambo(mambo_bluetooth_adress.islower(), use_wifi=False)
print("trying to connect")

success = mambo.connect(num_retries=3)
print(f""" connected >>>> {success}""")

if success:
    print("sleeping")
    mambo.smart_sleep(2)

    # print("getting status")
    # mambo.ask_for_state_update()
    # mambo.smart_sleep(2)
    # print("end of getting status")


    pic_success = mambo.take_picture()
 

    print("disconnect")
    mambo.disconnect()


    
