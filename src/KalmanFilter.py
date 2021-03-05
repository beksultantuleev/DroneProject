import numpy as np


class KalmanFilter:
    def __init__(self, A, B, C, D, Rw, Rv, X0, U0):
        """
        Give matrices for an estimator of the form

        xhat' = Axhat + Bu + L(y - yhat)
        yhat = Cxhat + Du

        X0: initial state matrix
        U0: control input matrix

        type of all matrices is np.array
        """
        self.A = A
        self.B = B
        self.C = C
        self.D = D

        self.X = X0
        self.U = U0

        # Initialize covariance matrices
        self.init_P()              # Error covariance
        self.Rw = Rw               # Process noise covariance
        self.Rv = Rv               # Measurement noise covariance

        # Initialize estimation gains
        self.L = self.update_L()

        # Measurement matrices
        self.Yhat = np.array([np.dot(self.A, self.X)+(np.dot(self.D, self.U))])

    def init_P(self):
        """
        Computes initial state covariance matrix from

        P[0] = E[X0*X0.T]

        NOTE: expected value of a constant is just the constant
        """
        self.P = np.dot(self.X, self.X.T)

    def update_L(self):
        """
        Computes L[k] from P[k] and state matrices

        L[k] = AP[K]C.T(Rv + CP[k]C.T)^-1
        """
        M = np.linalg.inv(self.Rv + np.dot(self.C, np.dot(self.P, self.C.T)))
        self.L = np.dot(self.A, np.dot(self.P, np.dot(self.C.T, M)))

        return None

    def update_estim(self, Y):
        """
        Computes next state estimate

        xhat[k+1] = Axhat[k] + Bu[k] + L[k](y[k] - yhat[k])
        P[k+1] = (A-L[k]C)P[k](A-L[k]C).T + Rv + L[k]RwL[k].T

        Y: numpy array of sensor values at timestep k
        """
        self.X = np.dot(self.A, self.X) + np.dot(self.B,
                                                 self.U) + np.dot(self.L, Y - self.Yhat)

        Acl = self.A - np.dot(self.L, self.C)
        self.P = np.dot(Acl, np.dot(self.P, Acl.T)) + self.Rv + \
            np.dot(self.L, np.dot(self.Rw, self.L))

        self.Yhat = np.dot(self.C, self.X) + np.dot(self.D, self.U)
        return None

    def get_state_estimate(self, Y, U):
        """
        Given sensor measurements and control input, return the state estimate
        """
        self.update_L()
        self.U = U
        self.update_estim(Y)

        return self.X.tolist()[0]


class MamboKalman(KalmanFilter):
    def __init__(self, X0, U0):
        """
        Initializes KalmanFilter for a parrot mambo

        Model is discrete-time and is based on a simplified view of the drone
        where input is linear velocity and the states are position only.
        The system is fully observable; we use the position estimation of the
        drone as a "sensor".

        states: x y z (coordinate positions)
        inputs: u v w (coordinate velocities)
        """
        X0 = np.matrix(X0).T
        U0 = np.matrix(U0).T
        self.dt = 0.5  # sample time in seconds. 2hz over WiFi.
        A = np.eye(len(X0))
        B = np.eye(len(X0)) * self.dt
        C = np.eye(len(X0))
        D = np.zeros((len(X0), len(X0)))
        Rw = np.eye(len(X0))  #made it 10
        Rv = np.eye(len(X0))  #made it 0.5

        super().__init__(A, B, C, D, Rw, Rv, X0, U0)


if __name__ == "__main__":
    # pos = [2,0,0, 1.9, 0,0]
    pos = [1.1, 1.2, 1.3]
    speed = [0.8, 0.1, 0.1]
    estimate = MamboKalman([0,0,0], [0,0,0])
    x = estimate.get_state_estimate(pos, speed)

    print(f"our UAV is here >>\tx\ty\tz\n\t\t\t{x}")
