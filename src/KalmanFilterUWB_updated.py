import numpy as np
from numpy.linalg import multi_dot

class KalmanFilterUWB:
    def __init__(self, q):
        '''"
        Give matrices for an estimator of the form
        _____
        q_pred = A*q + G*[ax, ay, az]  |ax+bu
        p_pred = A*Pup*A.T + GQG.T |c + du
        z = [x,y,z, x1,y1,z1]

        ____
        '''
        self.A = np.eye(np.size(q,0))
        self.G = np.eye(np.size(q,0)) * 0.5
        self.H = np.eye(6)
        self.R = np.eye(np.size(q,0))
        self.Q = np.eye(np.size(q,0))
        self.q = q
        
        self.S = []
        self.z = z
        
    def calculation(self, q, u, z, p_update_old, FLAG):
        self.q = q
        self.u = u
        self.z = z
        self.p_update_old = p_update_old
        self.FLAG = FLAG

        self.q_prediction = np.dot(self.A, self.q)+np.dot(self.G,self.u)
        self.p_prediction = multi_dot([self.A, self.p_update_old, self.A.T]) + multi_dot([self.G, self.Q ,self.G.T])

        if self.FLAG:
            self.s_update = np.dot( np.dot(self.H, self.p_prediction), self.H.T) + self.R
            self.W_gain = np.dot(np.dot(self.p_prediction, self.H), np.linalg.inv(self.s_update))
            self.q_update = self.q_prediction + np.dot(self.W_gain,  (self.z - np.dot(self.H,self.q_prediction)))
            self.p_update = (self.H - np.dot(np.dot(self.W_gain , self.H)), self.p_prediction)
        else:
            self.p_update = self.p_prediction
            self.q_update = self.q_prediction   
        self.S = [self.p_update, self.q_update]
        return self.S



# class MamboKalman(KalmanFilterUWB):
#     def __init__(self, q, u):
#         self.dt = 0.5

#         self.A = np.eye(len(q))
#         self.G = np.eye(len(q)) * self.dt
#         self.H = np.eye(len(q))
#         self.R = np.eye(len(q))
#         q = np.matrix(q).T
#         u = np.matrix(u).T

#         # q = [1,2,3]
#         # u = [0.4,0.3,0.5]


if __name__ == "__main__":
    q =np.ones((3, 1))
    p = np.zeros((3, 3))
    u = np.ones((3, 1)) # current speed_x, speed_y, speed_z
    z = np.ones((6, 1)) # current x,y,z and UWB x,y,z
    state = KalmanFilterUWB(q)
    UWBdata = 1
    if UWBdata: 
        FLAG=True
    else:
        FLAG = False

    p_update1, q_update1 = state.calculation(q,u,z,p,FLAG)
