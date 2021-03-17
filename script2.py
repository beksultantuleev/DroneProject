import numpy as np
import time

pos = np.array([3.5,2.5,1])
rotated_90_deg = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])

print(np.dot(rotated_90_deg, pos))
# def UWB_position_filter(given_position, prev_position):
#        # prev_position = given_position
#        difference = ((given_position[0] - prev_position[0])**2 +
#               (given_position[1] - prev_position[1])**2 +
#               (given_position[2] - prev_position[2])**2)**0.5
#        # if difference>0.1:
#        #        given_position = prev_position
#        return (difference)
#        #return given_position

# # giv_pos = [-0.08716263, 2.14139607, -0.25910652]
# giv_pos = [2.0397874, 2.54545619, 0.88413192]
# prev_pos = [1.26141207, 2.58196379, 1.91312271]
# print(UWB_position_filter(giv_pos, prev_pos))

# # while True:

# #        given_position = [np.random.randint(0,10),np.random.randint(0,10),np.random.randint(0,10)]
# #        print(f"given numbers {given_position}")
# #        print(f"after function {UWB_position_filter(given_position)}")

# #        time.sleep(1)
