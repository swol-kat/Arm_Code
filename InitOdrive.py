import odrive
import odrive.enums
import time
import math
import keyboard
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import robotJoint
import armKinematics
import json

print("finding Odrives")
odrives = odrive.find_any(find_multiple = 2)

# Multiple Odrive setup:
# Serial number for arm 1 lower and upper: 35593308029517
# Serial number for arm 1 shoulder: 35623373914701
axisDict = json.loads(open('axis_config.json', "r").read())

output_dict = {}
for odrive in odrives:  
    c = axisDict[str(odrive.serial_number)]
    
    output_dict[c['axis1']['name']] = robotJoint.robotJoint(odrive.axis1, c['axis1']['ratio'])
    output_dict[c['axis0']['name']] = robotJoint.robotJoint(odrive.axis1, c['axis0']['ratio'])

armVariables = [3.319, 3.125, 7.913, 9]

singleArm = armKinematics.Arm(output_dict['1 upper'], output_dict['1 upper'], output_dict['1 upper'], armVariables)

singleArm.calibrateArm()

singleArm.homeArm()

print("Setup Complete!")

