import time
import matplotlib.pyplot as plt
from simple_pid import PID


class DronePID:
    """
    Simple simulation of a water boiler which can heat up water
    and where the heat dissipates slowly over time
    """

    def __init__(self):
        self.desired_state = 0

    def update(self, pitch_angle, dt):

        # self.current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
        #                                 self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
        #                                 self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]
        # self.current_velocities = [self.mambo.sensors.speed_x,
        #                                self.mambo.sensors.speed_y,
        #                                self.mambo.sensors.speed_z]
        # self.current_state = self.kalmanfilter.get_state_estimate(self.current_measurement,
        #                                                               self.current_velocities)

        if pitch_angle > 0:
            self.desired_state += 1 * pitch_angle * dt
        elif pitch_angle <0:
            self.desired_state -= 1 * pitch_angle * dt

        # # some heat dissipation
        # self.desired_state -= 0.02 * dt
        # print(pitch_angle)
        return self.desired_state


if __name__ == '__main__':
    drone = DronePID()
    desired_state = drone.desired_state

    pid = PID(5, 0.01, 0.1, setpoint=desired_state)
    pid.output_limits = (-20, 20)

    start_time = time.time()
    last_time = start_time

    # keep track of values for plotting
    setpoint, y, x = [], [], []

    while time.time() - start_time < 5:
        current_time = time.time()
        dt = current_time - last_time
        # dt = 0.5

        power = pid(desired_state)
        desired_state = drone.update(power, dt)

        x += [current_time - start_time]
        y += [desired_state]
        setpoint += [pid.setpoint]

        if current_time - start_time > 1:
            pid.setpoint = 1

        last_time = current_time

    plt.plot(x, y, label='measured')
    plt.plot(x, setpoint, label='target')
    plt.xlabel('time')
    plt.ylabel('state points')
    plt.legend()
    # plt.show()