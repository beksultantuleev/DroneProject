from multiprocessing import Process, Pipe
import numpy as np
'''this is our UWB script'''


def UWBscript(child_conn):
    coordinates = [0, 0, 0]
    i = 0
    while i < 5:
        coordinates[0] = np.random.randint(1, 3)
        coordinates[1] = np.random.randint(4, 6)
        coordinates[2] = np.random.randint(7, 9)
        child_conn.send(coordinates)
        i += 1
    child_conn.close()


# if __name__ == '__main__':
#     UWBscript()
