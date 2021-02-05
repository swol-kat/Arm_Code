import math
import numpy as np
from kinematics import htm
import time
from copy import copy
from trajectory import quintic


class Arm:
    def __init__(self, lower_axis, upper_axis, shoulder_axis, arm_vars):
        self.lower_axis = lower_axis
        self.upper_axis = upper_axis
        self.shoulder_axis = shoulder_axis
        self.arm_vars = arm_vars

        self.targetPos = np.array([[0], [0], [0]])
        self.thetas = np.array([[0], [0], [0]])
        self.pos = np.array([[16.913], [3.125], [3.319]])
        self.vel = np.array([[0], [0], [0]])
        self.torque = np.array([[0], [0], [0]])

    def ikin(self, pos_vect):
        x, y, z = pos_vect.reshape(3)
        d1, d2, a2, a3 = self.arm_vars.values()
        # theta 1
        phi = math.atan2(y, x)
        r = math.hypot(x, y)
        alph = math.atan2(d2, math.sqrt(r ** 2 - d2 ** 2))
        t1 = phi - alph
        # theta 2 and 3
        s = z - d1
        u = math.sqrt(r ** 2 - d2 ** 2)
        D = (u ** 2 + s ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
        D = min((D, 1))
        t3 = math.atan2(- math.sqrt(1 - D ** 2), D)
        beta = math.atan2(s, r)
        gamma = math.atan2(a3*math.sin(t3),a2+a3*math.cos(t3))
        t2 = beta - gamma

        return np.array([t1, t2, t3]).reshape((3, 1))

    def send_to_pos(self, thetas):
        t1, t2, t3 = thetas.reshape(3)
        self.shoulder_axis.set_setpoint(t1)
        self.upper_axis.set_setpoint(t2)
        self.lower_axis.set_setpoint(t3)

    def go_to(self, target_pos, movement_time=.5):
        start_time = time.time()
        elapsed_time = time.time() - start_time
        start_pos = copy(self.pos)
        diff = target_pos - start_pos
        while elapsed_time <= movement_time:
            perc_move = quintic(elapsed_time / movement_time)

            new_target = diff * perc_move + start_pos

            thetas = self.ikin(new_target)
            print(thetas)
            self.send_to_pos(thetas)

            elapsed_time = time.time() - start_time
            time.sleep(.01)
        self.pos = target_pos

        print('reached')

    def set_current_limits(self, min_force, max_force):
        # know the jacobian and maths
        # F_tip = T * J(q)
        pass

    def jacobian(self, thetas=None):
        if not thetas:
            thetas = self.thetas

        pe = self.fwkin(thetas)

        J = np.zeros(6, 1)
        for i in range(3):
            tf = self.fwkin(thetas, joint=i, vector=False)
            z = tf[0:3, i]
            pi = tf[0:3, 3]

            Jp = np.cross(z, pe - pi)
            Jo = z
            Ji = np.concatenate((Jp, Jo), axis=0)
            J = np.concatenate((J, Ji), axis=1)

        return np.delete(J, 0, 1)

    def fwkin(self, thetas=None, joint=3, vector=True):
        """
        converts joint angles stored in self.thetas to workspace returns:
        [float]  returns a either a 4x4 matrix of the transform from joint 1 to the input joint or a 3x1 vector
                 depending on the vector variable
        """
        # dh table
        if not thetas:
            thetas = self.thetas

        dh_table = [[0, self.arm_vars['D1'], 0, - math.pi / 2],
                    [0, self.arm_vars['D2'], self.arm_vars['A2'], 0],
                    [0, 0, self.arm_vars['A3'], 0]]
        # identity matrix
        t_final = np.identity(4)
        # calculate fwkin
        for i in range(joint):
            params = dh_table[i]
            params[0] += thetas[i, 0]
            t_final = t_final @ htm(*params)
        # if vector retun just the pos var
        if vector:
            v = copy(t_final[0:3, 3])
            v.resize(3, 1)
            return v

        return t_final

    def update(self):
        # gets angle from each of the three joints

        t1 = self.shoulder_axis.get_pos()
        t2 = self.upper_axis.get_pos()
        t3 = self.lower_axis.get_pos()

        self.thetas = np.array([[t1], [t2], [t3]])

        self.pos = self.fwkin()

        self.vel = np.array([[self.shoulder_axis.get_vel()], [self.upper_axis.get_vel()], [self.lower_axis.get_vel()]])

        self.torque = np.array(
            [[self.shoulder_axis.get_torque()], [self.upper_axis.get_torque()], [self.lower_axis.get_torque()]])

    def home_arm(self):
        print("homing shoudler")
        self.shoulder_axis.run_manual_homing_routine()
        print("homing upper")
        self.upper_axis.run_manual_homing_routine()
        print("homing lower")
        self.lower_axis.run_manual_homing_routine()

    def calibrate_arm(self):
        self.shoulder_axis.calibrate_joint()
        self.upper_axis.calibrate_joint()
        self.lower_axis.calibrate_joint()

    def enable_arm(self):
        self.shoulder_axis.enable_joint()
        self.upper_axis.enable_joint()
        self.lower_axis.enable_joint()

    def fuck(self):
        self.shoulder_axis.fuck()
        self.upper_axis.fuck()
        self.lower_axis.fuck()