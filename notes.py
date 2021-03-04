import numpy as np
import scipy.linalg
dt = 0.5
A = np.array(
    [[1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0],
     [0.0, 0.0, 1.0]])
B = np.array(
    [[dt, 0.0, 0.0],
     [0.0, dt, 0.0],
     [0.0, 0.0, dt]])
Q = np.array(
    [[0.7, 0.0, 0.0],
     [0.0, 0.7, 0.0],
     [0.0, 0.0, 1]])
R = np.array(
    [[2, 0.0, 0.0],
     [0.0, 2, 0.0],
     [0.0, 0.0, 1]])

P = np.matrix(scipy.linalg.solve_discrete_are(A, B,
                                              Q, R))
print(P)