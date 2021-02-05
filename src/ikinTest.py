from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import time
import math
L1 = 7.913
L2 = 8

while(1):
    x = float(input("Type X Cordinate:"))
    y = float(input("Type Y Cordinate:"))
    # todo: kinematics test
    theta1 = math.acos((math.pow(L1, 2) + math.pow(x, 2) + math.pow(y, 2) - math.pow(L2, 2))/(2 * L1 * math.sqrt(math.pow(x, 2) + math.pow(y, 2)))) - math.atan2(x,y)
    theta2 = math.acos((math.pow(L1, 2) - math.pow(x, 2) - math.pow(y, 2) + math.pow(L2, 2))/(2 * L1 * L2)) + theta1 - math.pi
    theta2 *= -1.0
    t1x = L1 * math.sin(theta1) * -1.0
    t1y = L1 * math.cos(theta1)
    t2x = L2 * math.sin(theta2)
    t2y = L2 * math.cos(theta2)
    plt.plot([0, t1x],[0, t1y])
    plt.plot([t1x, t1x + t2x], [t1y, t1y + t2y])
    #plt.axis([-10, 10, -5, 15])
    print(theta1)
    print(theta2)
    plt.show()