from arm import Arm
from arm.joint import VirtualJoint


def setup():
    arm_variables = {'D1': 3.319, 'D2': 3.125, 'A2': 7.913, 'A3': 9.0}

    single_arm = Arm(VirtualJoint(), VirtualJoint(), VirtualJoint(), arm_variables)

    single_arm.calibrate_arm()

    single_arm.home_arm()

    single_arm.enable_arm()

    return {
        'right_arm':single_arm
    }

if __name__ == "__main__":
    arm = setup()