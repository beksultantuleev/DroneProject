import scipy.linalg
import numpy as np


class PositionController:
    def __init__(self, A, B, Q, R):
        """
        desired_state   : holds the current desired state of the drone in
                            the state format:
                            [x, y, z, dx/dt, dy/dt, dz/dt]
                            where dx/dt, dy/dt, dz/dt are velocities in the
                            x,y,z directions respectively.
        current_state   : holds the current state of the drone as estimated
                            by the KalmanFilter module.
        max_input_power : corresponds to the maximum power (normalized to 100
                            in the pyparrot module) that is permited in each
                            of the following commands:
                            [roll, pitch, yaw, vertical_movement]
        cmd_input       : list of commands (the u vector) for to get convergence
                            to desired state from current state."""
        self.desired_state = []
        self.current_state = []
        # self.necessary_input = []
        self.cmd_input = []

        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.P = np.matrix(scipy.linalg.solve_discrete_are(
            self.A, self.B, self.Q, self.R))

        self.K = np.matrix(scipy.linalg.inv(
            self.B.T*self.P*self.B+self.R) * (self.B.T*self.P*self.A))
        
        def get_eig(self):
            return scipy.linalg.eig(self.A-self.B*self.K)

    def get_current_input(self):
        """
        Return the current command input to the quadrotor.
        This is stored as a member of the PositionController object and
        is recalculated after every desired/current state update.
        """
        return self.cmd_input

    def set_current_state(self, current_state):
        """
        Called at every state update in a main loop.
            current_state: holds the current state of the drone.
                the state is x-y-z positions and x-y-z linear velocities.
        """
        self.current_state = current_state

    def set_desired_state(self, desired_state):
        """
        Called when the desired waypoint is updated in a main loop.
            desired_state: holds the desired x-y-z position of the drone as a
                3-element list.
        """
        self.desired_state = desired_state

    def calculate_cmd_input(self):
        """
        Determine x y z velocities required to converge to the desired state
        from the current state. These become roll, pitch, yaw "commands" in the
        pyparrot fly_direct() method.
        Our implementation drives a tracking error to zero. The generalized
        formulation of the control law is:
            u = -K_lqr * (x - x_d) + u_d
        where x_d is desired state and u_d is the input required to maintain
        the desired state. Inputs to the system are velocities, so the
        u_d will be the zero vector to maintain hover at desired coordinates.
        """
        x_er = np.subtract(self.current_state, self.desired_state)
        u = np.dot(-1 *self.K,  x_er).tolist()[0]

        self.cmd_input = u
        return self.get_current_input()

class MamboPositionController(PositionController):
    def __init__(self):

        self.dt = 0.5 # seconds; sample time (2hz on WiFi)
        # This system is in discrete time:
        A = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]])
        B = np.array([[self.dt, 0.0, 0.0],
                      [0.0, self.dt, 0.0],
                      [0.0, 0.0, self.dt]])
        Q = np.array([[1000, 0.0, 0.0],
                      [0.0, 1000, 0.0],
                      [0.0, 0.0, 1.0]])
        R = np.array([[0.5, 0.0, 0.0],
                      [0.0, 0.5, 0.0],
                      [0.0, 0.0, 1.0]])
        super().__init__(A, B, Q, R)

        self.max_input_power = [40, 40, 40, 40]
        self.max_velocity = 1.0 # m/s, I guess.

        def calculate_cmd_input(self):
            x_er = np.subtract(self.current_state, self.desired_state)
            u = np.dot(-1 *self.K, x_er).tolist()[0] # python list structure
            yaw = 0 # shouldn't have to yaw for our purposes.
            u_scaled = [0 for i in range(len(u))]

            # constraint checks:
            for i in range(len(u)):
                if u[i] > self.max_velocity:
                    u[i] = self.max_velocity
                if u[i] < -1 * self.max_velocity:
                    u[i] = -1 * self.max_velocity

                # scaling command input to power maximums:
                u_scaled[i] = u[i] / self.max_velocity * self.max_input_power[i]
            self.cmd_input = [u_scaled[0], u_scaled[1], yaw, u_scaled[2]]

            return self.get_current_input()


# test:
if __name__ == "__main__":
    mambo = MamboPositionController()

    mambo.set_desired_state([1, 0, 1])
    mambo.set_current_state([0, 0, 1])
    u = mambo.calculate_cmd_input()
    print('you need to input this:',u)
