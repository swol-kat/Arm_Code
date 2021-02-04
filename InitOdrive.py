import odrive
import odrive.enums
import time
import math
import keyboard
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from robotJoint import robotJoint
import armKinematics
import json

print("finding Odrives")
odrives = odrive.find_any(find_multiple = 2)

# Multiple Odrive setup:
# Serial number for arm 1 lower and upper: 35593308029517
# Serial number for arm 1 shoulder: 35623373914701
axisDict = json.load('axis_config.json')

output_dict = {}
for odrive in odrives:  
    c = axisDict[odrive.serial_number]
    
    output_dict[c['axis1']['name']] = robotJoint(odrive.axis1, c['axis1']['ratio'])
    output_dict[c['axis0']['name']] = robotJoint(odrive.axis1, c['axis0']['ratio'])

armVariables = [3.319, 3.125, 7.913, 8]

singleArm = armKinematics.arm(output_dict['1 upper'], output_dict['1 upper'], output_dict['1 upper'], armVariables)


print("Setup Complete!")
L1 = 7.913
L2 = 8
T = 0
while(1):
    x = 8 + 2 * math.cos(T)
    y = 9 + 2 * math.sin(T)
    # todo: kinematics test
    theta1 = math.acos((math.pow(L1, 2) + math.pow(x, 2) + math.pow(y, 2) - math.pow(L2, 2))/(2 * L1 * math.sqrt(math.pow(x, 2) + math.pow(y, 2)))) - math.atan2(x,y)
    theta2 = math.pi - math.acos((math.pow(L1, 2) - math.pow(x, 2) - math.pow(y, 2) + math.pow(L2, 2))/(2 * L1 * L2)) - theta1
    myDrive.axis0.controller.input_pos = theta1 * 9.0 / 2 / math.pi
    myDrive.axis1.controller.input_pos = theta2 * 9.0 / 2 / math.pi * -1.0
    time.sleep(0.01)
    T += 0.02
    #print(theta1)
    #print(theta2)
    #print(myDrive.axis0.controller.input_pos)
    #print(myDrive.axis1.controller.input_pos)
    #t1x = L1 * math.sin(theta1) * -1.0
    #t1y = L1 * math.cos(theta1)
    #t2x = L2 * math.sin(theta2)
    #t2y = L2 * math.cos(theta2)
    #plt.plot([0, t1x],[0, t1y])
    #plt.plot([t1x, t1x + t2x], [t1y, t1y + t2y])
    #plt.axis([-10, 10, -5, 15])
    #print(theta1)
    #print(theta2)
    #plt.show()
