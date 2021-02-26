# stage1.py
import time
import random

class Stage1:

    def stage1(self, queueS1, queueS2):
        print("stage1")
        lala = []
        lis = [1, 2, 3, 4, 5]
        for i in range(len(lis)):
            # to avoid unnecessary waiting
            if not queueS2.empty():
                msg = queueS2.get()    # get msg from s2
                print("! ! ! stage1 RECEIVED from s2:", msg)
                lala = [6, 7, 8] # now that a msg was received, further msgs will be different
            time.sleep(1) # work
            random.shuffle(lis)
            queueS1.put(lis + lala)             
        queueS1.put('s1 is DONE')