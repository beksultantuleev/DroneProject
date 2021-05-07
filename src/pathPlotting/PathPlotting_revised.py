import json
from typing import Annotated, Tuple
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
from adjustText import adjust_text


class PathDrawing:
    def __init__(self, date, in_3d):
        self.hitcounter = 0
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
        nameOfGraph = set()
        x = []
        y = []
        data = self.open_file()
        for i in data[draw_object]:
            y.append(i[0])
        for i in data["Time"]:
            x.append(i[0])
        try:
            for i in data["Title"]:
                nameOfGraph.add(i[0])
        except:
            nameOfGraph.add("no title")
        for i, sec in enumerate(x):
                # if  i<len(time)-1: #i != 0 and
                #     continue

                plt.annotate(f" {sec}s", xy=(x[i], y[i]), xytext=(x[i]+0.05,y[i]+0.05),  arrowprops = dict(arrowstyle="simple")) #works
                
        plt.title(list(nameOfGraph)[0])
        
        plt.plot(x, y, color, label=draw_object, marker='.')
        plt.legend()
        plt.xlabel("Seconds from Start")
        plt.ylabel("Distance ")
        plt.tight_layout()
        plt.style.use('ggplot')

    def draw(self, draw_object, color):
        self.hitcounter+=1
        nameOfGraph = set()
        is_local = set()
        x = []
        y = []
        z = []
        time = []
        data = self.open_file()
        try:
            for i in data["Title"]:
                nameOfGraph.add(i[0])
            for t in data["Local"]:
                is_local.add(t[0])
        except:
            nameOfGraph.add("no title")
            is_local.add("unknown")

        for i in data[draw_object]:
            if list(is_local)[0] and draw_object =="UWB":
                x.append(i[1])
                y.append(i[0])
            else:
                x.append(i[0])
                y.append(i[1])
            
            if self.in_3d:
                z.append(i[2])
        for i in data["Time"]:
            time.append(i[0])
        
        if self.in_3d: #3d graphs

            self.ax.plot3D(y, x,  z, color, label=draw_object, marker='.')
            plt.xlabel("X axis in meters")
            plt.ylabel("Y axis in meters")
            if list(is_local)[0]:
                plt.xlim(-1,1)
            else:
                # plt.ylim(0,6)
                plt.xlim(1,4)

            #labeling time of function for 3d
            for i, sec in enumerate(time):
                if  i<len(time)-1: #i != 0 and
                    continue
                # if list(is_local)[0]:
                #     self.ax.text(y[i], x[i], z[i]+0.005, f"{draw_object} end ({np.round(x[-1],1)},{np.round(y[-1], 1)}, {np.round(z[-1], 1)}) {sec}s", size=10, color='g')
                # else:
                self.ax.text(y[i], x[i], z[i]+0.005, f"{draw_object} end ({np.round(x[-1],1)},{np.round(y[-1], 1)}, {np.round(z[-1], 1)}) {sec}s", size=10, color='g')

                # if self.hitcounter<2:
                #     self.ax.text(x[i], y[i], z[i]+0.005, f"{draw_object} end ({np.round(x[-1],1)},{np.round(y[-1], 1)}, {np.round(z[-1], 1)}) {sec}s", size=10, color='g')
                # elif self.hitcounter<3:
                #     self.ax.text(x[i], y[i], z[i]+0.02, f"{draw_object} end ({np.round(x[-1],1)},{np.round(y[-1], 1)}, {np.round(z[-1], 1)}) {sec}s", size=10, color='g')
                # else:
                #     self.ax.text(x[i], y[i], z[i]+0.04, f"{draw_object} end ({np.round(x[-1],1)},{np.round(y[-1], 1)}, {np.round(z[-1], 1)}) {sec}s", size=10, color='g')
        
        else: #2d graphs

            plt.plot(x, y, color, label=draw_object, marker='.')
            plt.xlabel("X axis in meters")
            plt.ylabel("Y axis in meters")
            plt.ylim(0,6)
            if list(is_local)[0]:
                plt.ylim(-1, 1)
            
            
            #labeling time of function for 2d
            for i, sec in enumerate(time):
                if  i<len(time)-1: #i != 0 and
                    continue
                # np.random.seed(98)
                if self.hitcounter<2:
                    plt.annotate(f"{draw_object} end ({np.round(x[-1],1)},{np.round(y[-1], 1)}) {sec}s", xy=(x[i], y[i]), xytext=(x[i]-0.3,y[i]-0.3),  arrowprops = dict(arrowstyle="fancy")) #works
                elif self.hitcounter<3:
                    plt.annotate(f'{draw_object} end ({np.round(x[-1],1)},{np.round(y[-1], 1)}) {sec}s', xy=(x[i], y[i]), xytext=(x[i]-0.3,y[i]+0.3),  arrowprops = dict(arrowstyle="fancy")) #works
                else:
                    plt.annotate(f'{draw_object} end ({np.round(x[-1],1)},{np.round(y[-1], 1)}) {sec}s', xy=(x[i], y[i]), xytext=(x[i]-0.7,y[i]-1.2),  arrowprops = dict(arrowstyle="fancy")) #works
        


        plt.title(list(nameOfGraph)[0])
        plt.legend()


    def show(self):
        plt.tight_layout()
        plt.style.use('ggplot')
        plt.show()


if __name__ == "__main__":
    test = PathDrawing("Apr-27-2021-170255", False) #Apr-22-2021-174544 #Apr-27-2021-170255
    test.list_of_objects()
    test.draw_via_time("Distance", "red")

    # test.draw("IMU", "red")

    # test.draw("Kalman", "green")
    # test.draw("UWB", "blue")
    test.show()
