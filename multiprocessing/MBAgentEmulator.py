from multiprocessing import Process, Queue, Pipe
from UWBemulator import UWBscript
'''this should be our MB Agent'''

if __name__ == '__main__':
    parent_conn, child_conn = Pipe()
    p = Process(target=UWBscript, args=(child_conn,))
    p.start()
    coordinates = []
    i = 0
    while i < 5:
        coordinates = parent_conn.recv()
        # print(coordinates)
        i += 1
    print(coordinates)
