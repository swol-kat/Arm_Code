import odrive
import odrive.enums
import time
import math
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt

print("finding odrives")
od = odrive.find_any()

print("calibrating odrives")
od.axis1.clear_errors()
od.axis1.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE

while od.axis1.current_state != odrive.enums.AXIS_STATE_IDLE:
    time.sleep(0.01)

od.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
print("running loop")
while True:

    num_loops = 0

    time_list = []
    in_list = []
    pos_list = []
    vel_list = []
    torque_list = []
    lag_list = []

    frequency = int(input('Frequency: '))
    magnitude = float(input('Magnitude (rad): ')) / 2 / math.pi

    start_time = time.time()
    last_time = start_time

    while time.time() - start_time <= 5:
        #do 5s loop
        curr_time = time.time() - start_time
        time_list.append(curr_time)
        lag_list.append(curr_time - last_time)
        last_time = curr_time
        in_pos = math.sin(curr_time * frequency*2*math.pi) * magnitude
        od.axis1.controller.input_pos = in_pos
        in_list.append(in_pos * 2 * math.pi)
        pos_list.append(od.axis1.encoder.pos_estimate * 2 * math.pi)
        vel_list.append(od.axis1.encoder.vel_estimate)
        torque_list.append(od.axis1.motor.current_control.Iq_measured)
        num_loops += 1

    print(f"frequency: {frequency}")
    print(f"loop speed: {num_loops / 5}")
    fig = plt.figure()
    #plt.plot(lag_list, label = 'lag')
    plt.plot(time_list, in_list, label = 'input position')
    plt.plot(time_list, pos_list, label = 'curent position')
    #plt.plot(vel_list, label = 'velocity')
    #plt.plot(torque_list, label = 'torque')
    plt.legend()
    plt.xlabel('time (seconds)')
    plt.ylabel('position (radians)')
    plt.title(f'commanded position vs acutal position at {frequency}hz')
    plt.show()
