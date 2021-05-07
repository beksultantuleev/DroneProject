import sys
import os
import inspect
import numpy as np
from collections import Counter

currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from UWB_subscriber.subscriber import MqttSubscriber
import time
mqttSubscriber = MqttSubscriber("192.168.1.200", topic="Position1")
mqttSubscriber.start()
positions = []

def my_mode(sample):
    c = Counter(sample)
    return [k for k, v in c.items() if v == c.most_common(1)[0][1]]
full_list = []
test_list_x = []
for i in range(10): #15 was nice
    a = mqttSubscriber.pos
    if a==[]:
        time.sleep(0.5)
    time.sleep(0.08) #0.08 is fine
    if a:
        full_list.append(a) #works
    
    # if a:
    #     test_list_x.append(np.round(a[0], 2))
    # print(f'{np.round(a[0], 2)}, {a[1]}, {a[2]}')


mqttSubscriber.stop()
# print(full_list)
for i in full_list:
    test_list_x.append(np.round(i[0],2))
# print(test_list_x)

print(test_list_x)
print(f'mode is {my_mode(test_list_x)} \n and the avrg value is {np.mean(my_mode(test_list_x), axis=0)}')


# print(f"mode of list {my_mode(full_list)}")