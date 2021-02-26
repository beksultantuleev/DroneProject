# stage2.py
import time

class Stage2:

    def stage2(self, queueS1, queueS2):
        print("stage2")
        while True:
            msg = queueS1.get()    # wait till there is a msg from s1
            print("- - - stage2 RECEIVED from s1:", msg)
            if msg == 's1 is DONE ':
                break # ends loop
            time.sleep(1) # work
            queueS2.put("update lists")  