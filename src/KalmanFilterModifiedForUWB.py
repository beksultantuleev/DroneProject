import numpy as np


class KalmanFilter:
    def __init__(self, A, G, H, D, Q, R, z, u):
        """
        Give matrices for an estimator of the form

        xhat' = Axhat + Bu + L(y - yhat) | q_update = q_pred + W_gain(z-Hq_pred)
        yhat = Cxhat + Du

        X0: initial state matrix
        U0: control input matrix
        _________________
        xhat is q prediction
        yhat is p prediction  (probability)

        q_prediction is ( Axhat + Bu)=>> A is A
        xhat is q
        B is G
        u is u

        yhat is (H*q_prediction) or  (H*)
        _________________
        type of all matrices is np.array
        """
        self.A = A
        self.G = G #it was B
        self.H = H #is was C
        self.D = D #it was D

        self.z = z #it was X0
        self.u = u #was U0

        # Initialize covariance matrices
        self.init_P()              # Error covariance
        self.Q = Q #was Rw               # Process noise covariance
        self.R = R  #was Rv             # Measurement noise covariance

        # Initialize estimation gains
        self.W = self.update_W() #it was L instead of W

        # Measurement matrices
        self.Yhat = np.array([np.dot(self.A, self.z)+(np.dot(self.D, self.u))])

    def init_P(self):
        """
        Computes initial state covariance matrix from

        P[0] = E[X0*X0.T]

        NOTE: expected value of a constant is just the constant
        """
        self.P = np.dot(self.z, self.z.T)

    def update_W(self):
        """
        Computes L[k] from P[k] and state matrices

        L[k] = AP[K]C.T(Rv + CP[k]C.T)^-1
        """
        S_update = np.linalg.inv(self.R + np.dot(self.H, np.dot(self.P, self.H.T)))
        self.W = np.dot(self.A, np.dot(self.P, np.dot(self.H.T, S_update)))

        return None

    def update_estim(self, Y):
        """
        Computes next state estimate

        xhat[k+1] = Axhat[k] + Bu[k] + L[k](y[k] - yhat[k])
        P[k+1] = (A-L[k]C)P[k](A-L[k]C).T + Rv + L[k]RwL[k].T

        Y: numpy array of sensor values at timestep k
        """
        self.z = np.dot(self.A, self.z) + np.dot(self.G,
                                                 self.u) + np.dot(self.W, Y - self.Yhat)

        Acl = self.A - np.dot(self.W, self.H)
        self.P = np.dot(Acl, np.dot(self.P, Acl.T)) + self.R + \
            np.dot(self.W, np.dot(self.Q, self.W))

        self.Yhat = np.dot(self.H, self.z) + np.dot(self.D, self.u)
        return None

    def get_state_estimate(self, Y, U):
        """
        Given sensor measurements and control input, return the state estimate
        """
        self.update_W()
        self.u = U
        self.update_estim(Y)

        return self.z.tolist()[0]


class MamboKalman(KalmanFilter):
    def __init__(self, z, u):
        """
        Initializes KalmanFilter for a parrot mambo

        Model is discrete-time and is based on a simplified view of the drone
        where input is linear velocity and the states are position only.
        The system is fully observable; we use the position estimation of the
        drone as a "sensor".

        states: x y z (coordinate positions)
        inputs: u v w (coordinate velocities)
        """
        z = np.matrix(z).T
        u = np.matrix(u).T
        self.dt = 0.5  # sample time in seconds. 2hz over WiFi.
        A = np.eye(len(u))
        G = np.eye(len(u)) * self.dt
        H = np.eye(len(z))
        # H = np.array([[1.0, 0.0, 0.0],
        #                    [0.0, 1.0, 0.0],
        #                    [0.0, 0.0, 1.0],
        #                    [1.0, 0.0, 0.0],
        #                    [0.0, 1.0, 0.0],
        #                    [0.0, 0.0, 1.0]])
        D =np.zeros((len(z), len(z)))
        Q = np.eye(len(u))
        R = np.eye(len(z))
        # R = np.array([[0.5, 0, 0, 0, 0, 0],
        #                    [0, 0.5, 0, 0, 0, 0],
        #                    [0, 0, 0.5, 0, 0, 0],
        #                    [0, 0, 0, 0.4, 0, 0],
        #                    [0, 0, 0, 0, 0.4, 0],
        #                    [0, 0, 0, 0, 0, 100]])

        super().__init__(A, G, H, D, Q, R, z, u)


if __name__ == "__main__":
    # pos = [2,0,0, 1.9, 0,0]
    pos = [1, 1, 1]
    speed = [0.8, 0.1, 0.1]
    estimate = MamboKalman(pos, speed)
    x = estimate.get_state_estimate(pos, speed)

    print(f"our UAV is here >>\tx\ty\tz\n\t\t\t{x}")
