import numpy as np
from numpy.core.fromnumeric import size
c = 299792458

A_n1 = np.array([[0.00], [7.19], [2.15]])
A_n2 = np.array([[0.00], [3.62], [3.15]])
A_n3 = np.array([[0.00], [0.00], [2.15]])
A_n4 = np.array([[4.79], [1.85], [3.15]])
A_n5 = np.array([[4.79], [5.45], [2.15]])
A_n6 = np.array([[3.00], [9.35], [3.15]])
A_n = np.array([A_n3, A_n1, A_n2, A_n4, A_n5, A_n6])
toa = np.array([[5, 4, 3, 2, 1, 1]])
tdoa = toa[0][0] - toa
tdoa = tdoa[0][1:]  #we trimed tdoa
D = tdoa*c #D is 5x1
A_diff_one = np.array((A_n3[0][0]-A_n[:, 0]).T[0][1:], dtype='float')
A_diff_two = np.array((A_n3[1][0]-A_n[:, 1]).T[0][1:], dtype='float')
A_diff_three = np.array((A_n3[2][0]-A_n[:, 2]).T[0][1:], dtype='float')

A = 2 * np.array([A_diff_one, A_diff_two, A_diff_three, D])
# b = D**2 + np.linalg.norm(A_n3)**2 - np.sum(A_n[:, :]**2, 1) #original
# [1:]  # reduced last mat by 1 since D is 5x1
b = D**2 + np.linalg.norm(A_n3)**2 - np.sum(A_n[:, :]**2, 1)
x_t0 = np.dot(b, np.linalg.pinv(A))  # b*A or A*b?? A*b does not work

x_t0 = np.array([x_t0[1], x_t0[2], x_t0[3]])


#loop
f = np.zeros((n-1, 1))
del_f = np.zeros((n-1, 3))

for ii in range(2, 3):
    f[ii-1] = np.linalg.norm(x_t0-A_n[:, ii])-np.linalg.norm(x_t0-A_n[:, 1])
    del_f[ii-1, 1] = (x_t0[1]-A_n[1, ii])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, ii])
                                                        ) - (x_t0[1]-A_n[1, 1])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, 1]))
    del_f[ii-1, 2] = (x_t0[2]-A_n[2, ii])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, ii])
                                                        ) - (x_t0[2]-A_n[2, 1])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, 1]))
    del_f[ii-1, 3] = (x_t0[3]-A_n[3, ii])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, ii])
                                                        ) - (x_t0[3]-A_n[3, 1])*np.linalg.inv(np.linalg.norm(x_t0-A_n[:, 1]))


x_t = np.linalg.pinv(del_f)*(D-f) + x_t0
