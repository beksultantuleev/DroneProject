import numpy as np

rad = 5
rangeX = [-5,5]
# rangeY = 5
rangeZ = [1,3]
numberOfPoints = 5
testset = set()

for numbers in range(-rad, rad):
    if numbers**2+numbers**2<=rad**2:
        testset.add((numbers, numbers))
print(testset)

