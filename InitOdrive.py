import odrive
import odrive.enums
import time
import math
import keyboard
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from robotJoint import robotJoint
from armKinematics import Arm
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
    output_dict[c['axis1']['name']] = robotJoint(odrive.axis1, c['axis1']['ratio'])
    output_dict[c['axis0']['name']] = robotJoint(odrive.axis0, c['axis0']['ratio'])

armVariables = {'A1' : 3.319, 'D1' : 3.125, 'A2' : 7.913, 'A3' : 9.0}

singleArm = Arm(output_dict['1 lower'], output_dict['1 upper'], output_dict['1 shoulder'], armVariables)

singleArm.calibrateArm()

singleArm.homeArm()

singleArm.enableArm()

print("Setup Complete!")

