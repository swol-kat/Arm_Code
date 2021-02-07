from virtual_joint import VirtualJoint
from arm import Arm
import json
import numpy as np

arm_variables = {'D1': 3.319, 'D2': 3.125, 'A2': 7.913, 'A3': 9.0}

single_arm = Arm(VirtualJoint(),VirtualJoint(),VirtualJoint(), arm_variables, True)

single_arm.calibrate_arm()

single_arm.home_arm()

single_arm.enable_arm()

print("Setup Complete!")
