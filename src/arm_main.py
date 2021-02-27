from robot import Arm

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
from robot import Odrive_Controller, Threaded_Joint

odrive_pipe = namedtuple('odrive_pipe', ['to_worker', 'to_main', 'axis_0_name', 'axis_1_name'])

print('finding odrives')

def odrive_worker(serial, conn):
    search_serial = format(int(serial), 'x').upper()
    od = odrive.find_any(serial_number = search_serial)
    conn.send(0)
    while True:
        command = conn.recv()

        out_data = {}
        out_data['axis_0'] = {}
        out_data['axis_1'] = {}
        out_data['odrive'] = {}

        #run given functions on assigned axes if available
        if command['axis_0']['function']:
            command['axis_0']['function'](od.axis0, out_data['axis_0'])
        if command['axis_1']['function']:    
            command['axis_1']['function'](od.axis1, out_data['axis_1'])
        if command['odrive']:
            command['odrive'](od, out_data['odrive'])

        #index command is necessary. added in odrive handler
        out_data['index'] = command['index']
        
        #pos, curr commands automatically done. must be sent in control packets
        od.axis0.controller.input_pos = command['axis_0']['pos_command']
        od.axis1.controller.input_pos = command['axis_1']['pos_command']
        od.axis0.motor.config.current_lim = ['axis_0']['curr_command']
        od.axis1.motor.config.current_lim = ['axis_1']['curr_command']

        out_data['axis_0']['data'] = {'pos':od.axis0.encoder.pos_estimate, 'vel':od.axis0.encoder.vel_estimate, 'current':od.axis0.motor.current_control.Iq_measured}
        out_data['axis_1']['data'] = {'pos':od.axis1.encoder.pos_estimate, 'vel':od.axis1.encoder.vel_estimate, 'current':od.axis1.motor.current_control.Iq_measured}

        conn.send(out_data)

axis_dict = json.loads(open('axis_config.json', "r").read())

serials = list(axis_dict.keys())

process_list = []
odrive_controllers = []
joint_dict = {}

print('starting processes')

for serial in serials:
    to_worker, to_main = Pipe()
    joint_dict[axis_dict[serial]['axis0']['name']] = Threaded_Joint(axis_dict[serial]['axis0']['ratio'], 8.27 / 160)
    joint_dict[axis_dict[serial]['axis1']['name']] = Threaded_Joint(axis_dict[serial]['axis1']['ratio'], 8.27 / 160)
    odrive_controllers.append(Odrive_Controller(to_main, joint_dict[axis_dict[serial]['axis0']['name']], joint_dict[axis_dict[serial]['axis1']['name']]))
    process_list.append(Process(target=odrive_worker, args=(serial, to_worker, )))
    process_list[-1].start()
    good = to_main.recv() #just wait for thread to respond so we know that it found the odrive

#initialize arm variables - move to JSON file eventually
arm_variables = {'D1': 3.319, 'D2': 3.125, 'A2': 7.913, 'A3': 9.0}

#make arm objects
arm_dict = {}
arm_dict['right_arm'] = Arm(joint_dict['1 lower'], joint_dict['1 upper'], joint_dict['1 shoulder'], arm_variables)
arm_dict['left_arm'] = Arm(joint_dict['2 lower'], joint_dict['2 upper'], joint_dict['2 shoulder'], arm_variables)

#start gui worker - needs arm object
process_list.append(Process(target=gui_worker, args=(arm_dict['right_arm'], )))
process_list[-1].start()

# main program

quit_loop = False
while not quit_loop:
    
    for drive in odrive_pipes:
        drive.to_main.send(deepcopy(fake_odrive_packet))
    # do joint calculations here and form packets
    
    for drive in odrive_pipes:
        data = drive.to_main.recv()

for process in process_list:
    process.terminate()




