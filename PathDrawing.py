import json
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
# import matplotlib
# matplotlib.use('Agg')

class PathDrawing:
    def __init__(self, date, in_3d):
        self.date = "logs/" + date + ".json"
        self.in_3d = in_3d
        if self.in_3d:
            self.fig = plt.figure()
            self.ax = plt.axes(projection='3d')

    def open_file(self):
        with open(self.date) as file:
            data = json.load(file)
            return data

    def list_of_objects(self):
        data = self.open_file()
        for object in data:
            print(object)

    def draw_via_time(self, draw_object, color):
        x = []
        y = []
        data = self.open_file()
        for i in data[draw_object]:
            y.append(i[0])
        for i in data["Time"]:
            x.append(i[0])
        plt.plot(x, y, color, label=draw_object, marker='.')
        plt.legend()
        plt.xlabel("TIME")
        plt.ylabel("Distance ")
        plt.tight_layout()
        plt.style.use('ggplot')

    def draw(self, draw_object, color):
        x = []
        y = []
        z = []
        data = self.open_file()
        for i in data[draw_object]:
            x.append(i[0])
            y.append(i[1])
            if self.in_3d:
                z.append(i[2])
        if self.in_3d:
            self.ax.plot3D(x, np.array(y), z, color, label=draw_object, marker='.')
        else:
            plt.plot(x, np.array(y), color, label=draw_object, marker='.')
            
        plt.xlim(5, -5)
        plt.legend()
        plt.xlabel("X axis in meters")
        plt.ylabel("Y axis in meters")
        plt.tight_layout()
        plt.style.use('ggplot')


    def show(self):
        plt.show()


if __name__ == "__main__":
    test = PathDrawing("Mar-12-2021-161142", True)
    # test.list_of_objects()
    # test.draw_via_time("Distance", "red")

    test.draw("IMU", "red")
    test.draw("Kalman", "green")
    test.draw("UWB", "blue")
    test.show()
