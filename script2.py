import json
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

# fig = plt.figure()
# ax = plt.axes(projection='3d')
x = []
y = []
z = []

x1 = []
y1 = []
z1 = []
with open("logs/Mar-06-2021.json") as file:
    data = json.load(file)
    for i in data["Kalman"]:
        x.append(i[0])
        y.append(i[1])
        z.append(i[2])
    for i in data["IMU"]:
        x1.append(i[0])
        y1.append(i[1])
        z1.append(i[2])
# ax.plot3D(x, y, z, 'gray')
# ax.plot3D(x1, y1, z1, 'red')
plt.plot(x, y, "gray",label="Kalman")
plt.plot(x1, y1, "red", label="IMU")
plt.xlabel("meters")
plt.ylabel("meters")


plt.legend()
plt.show()