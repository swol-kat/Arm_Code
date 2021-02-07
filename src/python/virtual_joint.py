import time
import numpy as np

class VirtualJoint:
    def __init__(self, invert = False):
        self.pos = float(0)
        self.invert = invert

    def calibrate_joint(self):
        cal_time = 2
        step = .01
        for i in np.linspace(self.pos, cal_time, int((cal_time-self.pos/step))):
            self.set_setpoint(i)
            time.sleep(step)

    def enable_joint(self):
        return True

    def set_setpoint(self, angle):
        self.pos = float(angle)
        if self.invert:
            self.pos = -self.pos

    def set_torque(self, torque):  # set torque in N*M
        pass

    def fuck(self):
        self.set_setpoint(0)

    def get_setpoint(self):
        return self.pos

    def get_pos(self):
        """
            [float] pos of arm in rad
        """
        return self.pos

    def get_vel(self):
        """
            [float] velocity of arm in rad/s
        """
        return 0

    def get_torque(self):
        """
            [float] torquw of arm in Nm
        """
        return 0

    def run_manual_homing_routine(self):
        self.set_setpoint(0)
