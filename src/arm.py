import math
import numpy as np
from kinematics import htm
import time
from trajectory import quintic
from stick_plot import Plot
from copy import copy

class Arm:
    def __init__(self, lower_axis, upper_axis, shoulder_axis, arm_vars, plot=False):
        self.lower_axis = lower_axis
        self.upper_axis = upper_axis
        self.shoulder_axis = shoulder_axis
        self.arm_vars = arm_vars

        self.targetPos = np.array([[0], [0], [0]])
        self.thetas = np.array([[0], [0], [0]])
        self.pos = np.array([[0], [0], [0]])
        self.vel = np.array([[0], [0], [0]])
        self.torque = np.array([[0], [0], [0]])
        self.plot = False
        if plot:
            self.plot = Plot()

        self.update()

    def ikin(self, pos_vect, dog: True):
        x, y, z = pos_vect.reshape(3)
        d1, d2, a2, a3 = self.arm_vars.values()
        #print(f'x: {x}, y: {y}, z: {z}')
        #print(f'd1: {d1}, d2: {d2}, a2: {a2}, a3: {a3}')
        z -= d1
        L = math.sqrt(y**2 + x**2 - d2**2)
        # theta 1
        t1 = math.pi - math.atan2(y, x * -1.0) - math.acos(L/math.hypot(x, y))
        # theta 2 and 3
        a = -1.0 * math.atan2(z, L)
        if(dog):
            t2 = a + math.acos((a2**2 + z**2 + L**2 - a3**2)/(2 * math.hypot(z, L) * a2))
            t3 = a - math.acos((a3**2 + z**2 + L**2 - a2**2)/(2 * math.hypot(z, L) * a3))
        else:
            t2 = a - math.acos((a2**2 + z**2 + L**2 - a3**2)/(2 * math.hypot(z, L) * a2))
            t3 = a + math.acos((a3**2 + z**2 + L**2 - a2**2)/(2 * math.hypot(z, L) * a3))
        #print(f'z: {z}, L: {L}, t1: {t1}, a: {a}, t2: {t2}, t3: {t3}')

        return np.array([t1, t2, t3]).reshape((3, 1))

    def send_to_pos(self, thetas):
        t1, t2, t3 = thetas.reshape(3)
        self.shoulder_axis.set_setpoint(t1)
        self.upper_axis.set_setpoint(t2)
        self.lower_axis.set_setpoint(t3)

    def go_to_raw(self, target_pos, dog = True):
        thetas = self.ikin(target_pos, dog)
        self.send_to_pos(thetas)
        self.update()

    def go_to(self, target_pos, movement_time=.5, dog =True):
        start_time = time.time()
        elapsed_time = time.time() - start_time
        start_pos = copy(self.pos)
        diff = target_pos - start_pos
        while elapsed_time <= movement_time:
            perc_move = quintic(elapsed_time / movement_time)

            new_target = diff * perc_move + start_pos

            thetas = self.ikin(new_target, dog)
            self.send_to_pos(thetas)

            elapsed_time = time.time() - start_time
            self.update()
            time.sleep(.01)

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

    def fwkin(self, thetas=None, joint=4, vector=True):
        """
        converts joint angles stored in self.thetas to workspace returns:
        [float]  returns a either a 4x4 matrix of the transform from joint 1 to the input joint or a 3x1 vector
                 depending on the vector variable
        """
        # dh table
        if thetas is None:
            thetas = self.thetas

        t1, t2, t3 = thetas.reshape(3)

        dh_table = [[t1, self.arm_vars['D1'], 0, - math.pi / 2],
                    [0, self.arm_vars['D2'], 0, 0],
                    [t2, 0, self.arm_vars['A2'], 0],
                    [t3, 0, self.arm_vars['A3'], 0]]
        # identity matrix
        t_final = np.identity(4)
        # calculate fwkin
        for i in range(joint):
            params = dh_table[i]
            t_final = t_final @ htm(*params)
        # if vector retun just the pos var
        if vector:
            return t_final[0:3, 3].reshape(3, 1)

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

        if self.plot:
            self.plot.plot(self)

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
