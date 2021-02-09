import odrive
import odrive.enums
import math
import time

class Joint:
    def __init__(self, odrive_axis, gear_ratio):
        self.odrive_axis = odrive_axis
        self.gearRatio = gear_ratio
        self.torque_constant = odrive_axis.motor.config.torque_constant

    def calibrate_joint(self):
        self.odrive_axis.clear_errors()
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        t_start = time.time()
        while self.odrive_axis.current_state != odrive.enums.AXIS_STATE_IDLE and t_start + 10 > time.time():
            time.sleep(0.1)

    def enable_joint(self):
        self.odrive_axis.clear_errors()
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

    def set_setpoint(self, angle):
        self.odrive_axis.controller.input_pos = angle * self.gearRatio / 2 / math.pi

    def set_torque(self, torque):  # set torque in N*M
        self.odrive_axis.motor.config.current_lim = torque / self.torque_constant

    def fuck(self):
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_IDLE

    def get_setpoint(self):
        return self.odrive_axis.controller.input_pos / self.gearRatio * 2 * math.pi

    def get_pos(self):
        """
            [float] pos of arm in rad
        """
        return 2 * math.pi * self.odrive_axis.encoder.pos_estimate / self.gearRatio

    def get_vel(self):
        """
            [float] velocity of arm in rad/s
        """
        return 2 * math.pi * self.odrive_axis.encoder.vel_estimate / self.gearRatio

    def get_torque(self):
        """
            [float] torquw of arm in Nm
        """
        return self.odrive_axis.motor.current_control.Iq_measured * self.torque_constant

    def run_manual_homing_routine(self):
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        self.odrive_axis.encoder.set_linear_count(0)
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrive_axis.controller.input_pos = 0.0
        """
        while True:
            if keyboard.is_pressed('left'):
                self.odrive_axis.controller.input_pos += 0.01
            if keyboard.is_pressed('right'):
                self.odrive_axis.controller.input_pos -= 0.01
            if keyboard.is_pressed('space'):
                break
            time.sleep(0.05)
        """
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_IDLE
        self.odrive_axis.encoder.set_linear_count(0)
        time.sleep(1)
