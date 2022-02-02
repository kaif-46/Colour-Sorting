import numpy as np
import math

a = np.array([7,9]) #Co-ordinates of white ball
b = np.array([3,3]) #Co-ordinates of the ball
c = np.array([0,0]) #Co-ordinates of the pocket


#  function for mapping a range of value to another range values
def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

###########################code for calculating angle and probability based on 2nd rule ##################################
ba = a - b
bc = c - b
cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
angle = np.arccos(cosine_angle)
k=np.degrees(angle)
print("angle0",k)
n=180-k
print("fangle",n)
y = _map(n, 0, 90, 100, 0)
print(y)




############################ code for calculating distance to pocket and based on 1st rule  ##############################
d1 = math.dist(a, b)
d2 = math.dist(b, c)
prob = (1/(d1+d2))*100
print (prob)

############################ Code for calculating total probabilty  ##############################
print("Total probability of pocketing")
print(prob*y)

import math


def getAngle(a, b, c):
    ang = math.degrees(math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(a[1] - b[1], a[0] - b[0]))
    return ang + 360 if ang < 0 else ang


print(getAngle((7, 9), (3, 3), (0, 0)))