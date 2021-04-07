import numpy as np
import time
import random


radius = 3
rangeX = (-10, 10)
rangeY = (-10, 10)
qty = 5  # or however many points you want

deltas = set()
for x in range(-radius, radius+1):
    for y in range(-radius, radius+1):
        if x*x + y*y <= radius*radius:
            deltas.add((x,y))
print(deltas)
randPoints = []
excluded = set()
i = 0
while i<qty:
    x = random.randrange(*rangeX)
    y = random.randrange(*rangeY)
    if (x,y) in excluded: continue
    randPoints.append((x,y))
    i += 1
    excluded.update((x+dx, y+dy) for (dx,dy) in deltas)
# print (randPoints)
