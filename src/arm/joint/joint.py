import odrive
import odrive.enums
import math
import time


class Joint:
    def __init__(self, odrive_axis, gear_ratio):
        self.odrive_axis = odrive_axis
        self.gear_ratio = gear_ratio
        self.torque_constant = odrive_axis.motor.config.torque_constant

    def calibrate_joint(self):
        self.odrive_axis.clear_errors()
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        t_start = time.time()
        while self.odrive_axis.current_state != odrive.enums.AXIS_STATE_IDLE:
            time.sleep(0.1)

    def enable_joint(self):
        self.odrive_axis.clear_errors()
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

    def set_setpoint(self, angle):
        self.odrive_axis.controller.input_pos = angle * self.gear_ratio / 2 / math.pi

    def set_torque(self, torque):  # set torque in in*lb
        self.odrive_axis.motor.config.current_lim = torque / self.torque_constant / 8.8507 / self.gear_ratio

    def fuck(self):
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_IDLE

    def get_torque_limit(self):
        return self.odrive_axis.motor.config.current_lim * self.torque_constant * 8.8507 * self.gear_ratio

    def get_setpoint(self):
        return self.odrive_axis.controller.input_pos / self.gear_ratio * 2 * math.pi

    def get_pos(self):
        """
            [float] pos of arm in rad
        """
        return 2 * math.pi * self.odrive_axis.encoder.pos_estimate / self.gear_ratio

    def get_vel(self):
        """
            [float] velocity of arm in rad/s
        """
        return 2 * math.pi * self.odrive_axis.encoder.vel_estimate / self.gear_ratio

    def get_torque(self):
        """
            [float] torquw of arm in in*lb
        """
        return self.odrive_axis.motor.current_control.Iq_measured * self.torque_constant * 8.8507 * self.gear_ratio * -1.0 #that negative shouldnt be there

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
        #time.sleep(1)

    def get_error(self):
        return {
            'motor': hex(self.odrive_axis.motor.error),
            'encoder': hex(self.odrive_axis.encoder.error),
            'controller': hex(self.odrive_axis.controller.error)
        }
    
    def motor_configuration(self, encoder_cs_pin):
        self.odrive_axis.encoder.config.mode = 257
        self.odrive_axis.encoder.config.cpr = 16384
        self.odrive_axis.encoder.config.abs_spi_cs_gpio_pin = encoder_cs_pin
        self.odrive_axis.motor.config.pole_pairs = 20
        self.odrive_axis.motor.config.torque_constant = 8.27/160
        self.odrive_axis.motor.config.current_lim = 10.0
        self.odrive_axis.motor.config.current_lim_margin = 1000
        self.odrive_axis.motor.config.torque_lim = 10000
        self.odrive_axis.controller.config.enable_vel_limit = True
        self.odrive_axis.controller.config.control_mode = 3
        self.odrive_axis.controller.config.pos_gain = 25.0
        self.odrive_axis.controller.config.vel_gain = 0.17
        self.odrive_axis.controller.config.vel_integrator_gain = 0.33
        self.odrive_axis.controller.config.vel_limit = 3.0
        self.odrive_axis.controller.config.vel_limit_tolerance = 999999999
        self.odrive_axis.controller.config.vel_ramp_rate = 2.5
        self.odrive_axis.controller.config.torque_ramp_rate = 0.01
        self.odrive_axis.controller.config.inertia = 0.0
        
    def start_calibration(self):
        self.odrive_axis.clear_errors()
        self.odrive_axis.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    def is_calibration_complete(self):
        return self.odrive_axis.current_state == odrive.enums.AXIS_STATE_IDLE