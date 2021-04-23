print('start')
import odrive
import odrive.enums
import time
from multiprocessing import Process, Pipe
from copy import deepcopy
from collections import namedtuple
import json
import matplotlib as plot
from gui.gui import gui_worker
from robot import Robot, Arm, VirtualJoint, Threaded_Joint, Odrive_Controller

odrive_pipe = namedtuple('odrive_pipe', ['to_worker', 'to_main', 'axis_0_name', 'axis_1_name'])

# setup 
def setup():
    process_list = []
    odrive_controllers = []
    joint_dict = {}
    
    print("Loading Axis Data")
    axis_dict, serials = load_axis_data()
    
    print("Finding Odrives")
    for serial in serials:
        to_worker, to_main = Pipe()
        joint_dict[axis_dict[serial]['axis0']['name']] = Threaded_Joint(axis_dict[serial]['axis0']['ratio'], 8.27 / 160)
        joint_dict[axis_dict[serial]['axis1']['name']] = Threaded_Joint(axis_dict[serial]['axis1']['ratio'], 8.27 / 160)
        odrive_controllers.append(Odrive_Controller(to_main, joint_dict[axis_dict[serial]['axis0']['name']], joint_dict[axis_dict[serial]['axis1']['name']]))
        process_list.append(Process(target=odrive_worker, args=(serial, to_worker, )))
        process_list[-1].start()
        good = to_main.recv() #just wait for thread to respond so we know that it found the odrive
    
    print("Loading Arm Variables")
    #initialize arm variables - move to JSON file eventually
    arm_variables = load_arm_vars()
    print("Creating Arm Objects")
    arm_dict = create_arms(arm_variables)
        
def odrive_worker(serial, conn):
    search_serial = format(int(serial), 'x').upper()
    print(f'searching for odrive {search_serial}')
    od = odrive.find_any(serial_number = search_serial)
    print(f'found odrive {search_serial}')
    conn.send(0)
    while True:
        command = conn.recv()
        
        out_data = {}
        out_data['axis_0'] = {}
        out_data['axis_1'] = {}
        out_data['odrive'] = {}

        #run given functions on assigned axes if available
        if 'command' in command['axis_0']:
            #print('axis 0 command')
            command['axis_0']['command'](od.axis0, out_data['axis_0'])
        if 'command' in command['axis_1']: 
            #print('axis 1 command')
            command['axis_1']['command'](od.axis1, out_data['axis_1'])
        if 'command' in command:
            #print('odrive command')
            command['command'](od, out_data['odrive'])

        #index command is necessary. added in odrive handler
        out_data['index'] = command['index']
        
        #pos, curr commands automatically done. must be sent in control packets
        od.axis0.controller.input_pos = command['axis_0']['pos_command']
        od.axis1.controller.input_pos = command['axis_1']['pos_command']
        od.axis0.motor.config.current_lim = command['axis_0']['curr_command']
        od.axis1.motor.config.current_lim = command['axis_1']['curr_command']

        out_data['axis_0']['data'] = {'pos':od.axis0.encoder.pos_estimate, 'vel':od.axis0.encoder.vel_estimate, 'current':od.axis0.motor.current_control.Iq_measured}
        out_data['axis_1']['data'] = {'pos':od.axis1.encoder.pos_estimate, 'vel':od.axis1.encoder.vel_estimate, 'current':od.axis1.motor.current_control.Iq_measured}

        conn.send(out_data)

def load_axis_data():
    axis_dict = json.loads(open('axis_config_pack2.json', "r").read())
    serials = list(axis_dict.keys())
    return axis_dict, serials

def load_arm_vars():
    #initialize arm variables - move to JSON file eventually
    return {'D1': 3.319, 'D2': 3.125, 'A2': 7.913, 'A3': 7.913}

def create_arms(arm_variables):
    arm_dict = {}
    arm_dict['front_right'] = Arm(VirtualJoint(-9,8.27 / 160), VirtualJoint(-9,8.27 / 160), VirtualJoint(-5,8.27 / 160), arm_variables, 1)
    arm_dict['back_right'] = Arm(VirtualJoint(-9,8.27 / 160), VirtualJoint(-9,8.27 / 160), VirtualJoint(-5,8.27 / 160), arm_variables, 4)
    arm_dict['front_left'] = Arm(joint_dict['4 lower'], joint_dict['4 upper'], joint_dict['4 shoulder'], arm_variables, 2)
    arm_dict['back_left'] = Arm(VirtualJoint(-9,8.27 / 160), VirtualJoint(-9,8.27 / 160), VirtualJoint(-5,8.27 / 160), arm_variables, 3)
    return arm_dict

print('settings setup')

for controller in odrive_controllers:
    controller.set_odrive_params()

for controller in odrive_controllers:
    controller.send_packet()

for controller in odrive_controllers:
    controller.block_for_response()

#now do homing

print('calibrating motors')

#just calibrate one arm for arjun

joint_dict['4 upper'].calibrate()
joint_dict['4 lower'].calibrate()
joint_dict['4 shoulder'].calibrate()

cal_incomplete = True

while cal_incomplete:
    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()
    if joint_dict['4 upper'].is_calibration_complete() and joint_dict['4 lower'].is_calibration_complete() and joint_dict['4 shoulder'].is_calibration_complete():
        cal_incomplete = False

'''
print('homing motors')

joint_dict['4 upper'].home()
joint_dict['4 lower'].home()
joint_dict['4 shoulder'].home()

cal_incomplete = True

while cal_incomplete:
    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()
    if joint_dict['4 upper'].is_home_complete() and joint_dict['4 lower'].is_home_complete() and joint_dict['4 shoulder'].is_home_complete():
        cal_incomplete = False
'''
print('set motors in zero position')

time.sleep(3)

joint_dict['4 upper'].set_zero()
joint_dict['4 lower'].set_zero()
joint_dict['4 shoulder'].set_zero()

for controller in odrive_controllers:
    controller.send_packet()
for controller in odrive_controllers:
    controller.block_for_response()

print('enable motors')

joint_dict['4 upper'].enable()
joint_dict['4 lower'].enable()
joint_dict['4 shoulder'].enable()

for controller in odrive_controllers:
    controller.send_packet()
for controller in odrive_controllers:
    controller.block_for_response()

print('running loop')

#start gui worker - needs arm object
#process_list.append(Process(target=gui_worker, args=(None, )))
#process_list[-1].start()

#------------------------------------arjun do stuff here ---------------------------------------------
robot = Robot(arm_dict)
robot.boot()

# main program

try:
    while True:
        robot.loop()
        for controller in odrive_controllers:
            controller.send_packet()
        for controller in odrive_controllers:
            controller.block_for_response()
            
            
except(SystemExit):
    print('i die')

    joint_dict['4 upper'].disable()
    joint_dict['4 lower'].disable()
    joint_dict['4 shoulder'].disable()

    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()
    for process in process_list:
        process.terminate()
    #TODO: close serial port on reciever




