import numpy as np

# a = np.array([[1,12],[2,13],[3,14],[4,15],[5,16],[6,17],[7,18],[8,19]])

# print(np.mean(a,0))
c = []
for i in range(3):
    b = [np.random.randint(0,10),np.random.randint(0,10),np.random.randint(0,10)]
    c.append(b)
print(np.array(c))
print(np.mean(np.array(c), 0))
print(np.mean(np.array(c), 0)[0])
