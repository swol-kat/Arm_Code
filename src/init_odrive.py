import odrive
import odrive.enums

from src.robot.arm.joint import Joint
from src.robot.arm import Arm
import json


def setup():
    print("finding Odrives")
    odrives = odrive.find_any(find_multiple=3)

    print("found Odrives")

    # Multiple Odrive setup:
    axis_dict = json.loads(open('robot/axis_config.json', "r").read())
    
    output_dict = {}
    for od in odrives:
        c = axis_dict[str(od.serial_number)]
        output_dict[c['axis1']['name']] = Joint(od.axis1, c['axis1']['ratio'])
        output_dict[c['axis0']['name']] = Joint(od.axis0, c['axis0']['ratio'])

    arm_variables = {'D1': 3.319, 'D2': 3.125, 'A2': 7.913, 'A3': 9.0}

    arm_dict = {}

    arm_dict['right_arm'] = Arm(output_dict['1 lower'], output_dict['1 upper'], output_dict['1 shoulder'], arm_variables)

    arm_dict['left_arm'] = Arm(output_dict['2 lower'], output_dict['2 upper'], output_dict['2 shoulder'], arm_variables)

    for name, arm in arm_dict.items():
        print(f"calibrating {name}")
        arm.calibrate_arm()
        print(f"homing {name}")
        arm.home_arm()
        print(f"enabling {name}")
        arm.enable_arm()
    
    print("Setup Complete!")

    return arm_dict

def print_errors(arms):
    for name, arm in arms.items():
        if(arm.shoulder_axis.odrive_axis.error):
            print(f'error on {name} shoulder axis: {hex(arm.shoulder_axis.odrive_axis.error)}')
        if(arm.upper_axis.odrive_axis.error):
            print(f'error on {name} upper axis: {hex(arm.upper_axis.odrive_axis.error)}')
        if(arm.lower_axis.odrive_axis.error):
            print(f'error on {name} lower axis: {hex(arm.lower_axis.odrive_axis.error)}')

def config_params(arms):
    for name, arm in arms.items():
        arm.shoulder_axis.motor_configuration()
        arm.lower_axis.motor_configuration()
        arm.upper_axis.motor_configuration()
    #how save?


if __name__ == '__main__':
    arms = setup()
