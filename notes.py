# main.py
from multiprocessing import Process, Queue
from UWBscript import *
from ModelBasedAgent_UWB import ModelBasedAgentUWB



s1= main()
s2= ModelBasedAgentUWB

# S1 to S2 communication
# queueS1 = Queue()  # s1.stage1() writes to queueS1

# S2 to S1 communication
queueS2 = Queue()  # s2.stage2() writes to queueS2

# start s2 as another process
s2 = Process(target=s2.main, args=(queueS2))
s2.daemon = True
s2.start()     # Launch the stage2 process

s1.stage1( queueS2) # start sending stuff from s1 to s2 
s2.join() # wait till s2 daemon finishes