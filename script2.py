import numpy as np
import time

rotation_matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])

pos = [1,0,1]

result = list(np.dot(rotation_matrix, pos))
print(result)
