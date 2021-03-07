import json
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


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

    def draw(self, draw_object, color):
        x = []
        y = []
        z = []          
        data = self.open_file()
        for i in data[draw_object]:
            x.append(i[0])
            y.append(i[1])
            z.append(i[2])
        if self.in_3d:
            self.ax.plot3D(x, y, z, color)
        else:
            plt.plot(x, y, color, label=draw_object)
        plt.xlabel("meters")
        plt.ylabel("meters")
        plt.legend()

    def show(self):
        plt.show()


if __name__ == "__main__":
    test = PathDrawing("Mar-06-2021", False)
    # test.list_of_objects()
    test.draw("IMU", "red")
    test.draw("Kalman", "black")
    test.show()
