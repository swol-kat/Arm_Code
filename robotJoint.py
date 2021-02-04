import odrive
import odrive.enums
import math
import time
import keyboard

class RobotJoint:
    def __init__(self, odriveAxis, gearRatio):
        self.odriveAxis = odriveAxis
        self.gearRatio = gearRatio
        self.torque_constant = odriveAxis.motor.config.torque_constant

    def calibrateJoint(self): #TODO: add homing sequence
        self.odriveAxis.clear_errors()
        self.odriveAxis.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.odriveAxis.current_state != odrive.enums.AXIS_STATE_IDLE:
            time.sleep(0.1)

    def enableJoint(self):
        self.odriveAxis.clear_errors()
        self.odriveAxis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

    def setSetpoint(self, angle):
        self.odriveAxis.controller.input_pos = angle * self.gearRatio / 2 / math.pi

    def setTorque(self, torque): #set torque in N*M
        self.odriveAxis.motor.config.current_lim = torque / self.torque_constant
    
    def fuck(self):
        self.odriveAxis.requested_state = odrive.enums.AXIS_STATE_IDLE
    
    def get_setpoint(self):
        return self.odriveAxis.controller.input_pos / self.gearRatio * 2 * math.pi

    def get_pos(self): 
        '''
            [float] pos of arm in rad
        '''
        return 2 * math.pi * self.odriveAxis.encoder.pos_estimate / self.gearRatio 

    def get_vel(self): 
        '''
            [float] velocity of arm in rad/s
        '''
        return 2 * math.pi * self.odriveAxis.encoder.vel_estimate / self.gearRatio

    def get_torque(self):
        '''
            [float] torquw of arm in Nm
        '''
        return self.odriveAxis.motor.current_control.Iq_measured * self.torque_constant
    
    def runManualHomingRoutine(self):
        self.odriveAxis.requested_state = odrive.enums.AXIS_STATE_IDLE
        self.odriveAxis.encoder.set_linear_count(0)
        self.odriveAxis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odriveAxis.controller.input_pos = 0.0
        while(1):
            if keyboard.is_pressed('a'):
                self.odriveAxis.controller.input_pos += 0.01
            if keyboard.is_pressed('d'):
                self.odriveAxis.controller.input_pos -= 0.01
            if keyboard.is_pressed('space'):
                break
            time.sleep(0.05)
        self.odriveAxis.requested_state = odrive.enums.AXIS_STATE_IDLE
        self.odriveAxis.encoder.set_linear_count(0)