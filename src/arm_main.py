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

axis_data = namedtuple('axis_data', ['pos_now', 'vel_now', 'curr_now'])
axis_command = namedtuple('axis_command', ['pos', 'curr_lim'])

odrive_command = namedtuple('odrive_command', ['axis_0_command', 'axis_1_command'])
odrive_data = namedtuple('odrive_data', ['axis_0_data', 'axis_1_data'])

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

        #run given functions on assigned axes
        if command['axis_0']['function']:
            command['axis_0']['function'](od.axis0, out_data['axis_0'])
        if command['axis_1']['function']:    
            command['axis_1']['function'](od.axis1, out_data['axis_1'])
        if command['odrive']:
            command['odrive'](od, out_data['odrive'])

        #index command is necessary
        out_data['index'] = command['index']
        
        #pos, curr commands automatically done. must be sent
        od.axis0.controller.input_pos = command['axis_0']['pos_command']
        od.axis1.controller.input_pos = command['axis_1']['pos_command']
        od.axis0.motor.config.current_lim = ['axis_0']['curr_command']
        od.axis1.motor.config.current_lim = ['axis_1']['curr_command']

        out_data['axis_0']['data'] = {'pos':od.axis0.encoder.pos_estimate, 'vel':od.axis0.encoder.vel_estimate, 'current':od.axis0.motor.current_control.Iq_measured}
        out_data['axis_1']['data'] = {'pos':od.axis1.encoder.pos_estimate, 'vel':od.axis1.encoder.vel_estimate, 'current':od.axis1.motor.current_control.Iq_measured}

        conn.send(out_data)

axis_dict = json.loads(open('axis_config.json', "r").read())

serials = list(axis_dict.keys())

odrive_pipes = []
process_list = []

print('starting processes')

for serial in serials:
    to_worker, to_main = Pipe()
    odrive_pipes.append(odrive_pipe(to_worker, to_main, axis_dict[serial]['axis0']['name'], axis_dict[serial]['axis1']['name']))
    process_list.append(Process(target=odrive_worker, args=(serial, odrive_pipes[-1].to_worker, )))
    process_list[-1].start()
    good = odrive_pipes[-1].to_main.recv()

#make arm object

#start gui worker - needs arm object
process_list.append(Process(target=gui_worker, args=(None, )))
process_list[-1].start()

# main program
fake_data = axis_command(0, 0)
fake_odrive_packet = odrive_command(deepcopy(fake_data),deepcopy(fake_data))

quit_loop = False
while not quit_loop:
    
    for drive in odrive_pipes:
        drive.to_main.send(deepcopy(fake_odrive_packet))
    # do joint calculations here and form packets
    
    for drive in odrive_pipes:
        data = drive.to_main.recv()

for process in process_list:
    process.terminate()




