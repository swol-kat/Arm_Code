print('start')
import odrive
import odrive.enums
import time
from multiprocessing import Process, Pipe
from copy import deepcopy
from collections import namedtuple
import json

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

        od.axis0.controller.input_pos = command.axis_0_command.pos
        od.axis1.controller.input_pos = command.axis_1_command.pos
        od.axis0.motor.config.current_lim = command.axis_0_command.curr_lim
        od.axis1.motor.config.current_lim = command.axis_1_command.curr_lim

        axis0_data = axis_data(od.axis0.encoder.pos_estimate, od.axis0.encoder.vel_estimate, od.axis0.motor.current_control.Iq_measured)
        axis1_data = axis_data(od.axis1.encoder.pos_estimate, od.axis1.encoder.vel_estimate, od.axis1.motor.current_control.Iq_measured)

        data = odrive_data(axis0_data, axis1_data)

        conn.send(data)



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


# main program

num_loops = 0
start_time = time.time()
run_time = 5

print('starting test')

fake_data = axis_command(0, 0)
fake_odrive_packet = odrive_command(deepcopy(fake_data),deepcopy(fake_data))

while start_time + run_time > time.time():
    
    for drive in odrive_pipes:
        drive.to_main.send(deepcopy(fake_odrive_packet))
    for drive in odrive_pipes:
        data = drive.to_main.recv()
    num_loops += 1

print(f'Loop Frequency: {num_loops / run_time}')

for process in process_list:
    process.terminate()




