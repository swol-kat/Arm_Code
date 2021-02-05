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
        self.armVars = arm_vars

        self.targetPos = np.array([[0], [0], [0]])
        self.thetas = np.array([[0], [0], [0]])
        self.pos = np.array([[6], [6], [6]])
        self.vel = np.array([[0], [0], [0]])
        self.accel = np.array([[0], [0], [0]])
        self.torque = np.array([[0], [0], [0]])

        self.update()

    def ikin_2d(self, pos_vect):
        x = pos_vect[0][0]
        y = pos_vect[1][0]
        theta1 = math.acos(
            (math.pow(self.armVars['A2'], 2) + math.pow(x, 2) + math.pow(y, 2) - math.pow(self.armVars['A3'], 2)) / (
                    2 * self.armVars['A2'] * math.sqrt(math.pow(x, 2) + math.pow(y, 2)))) - math.atan2(x, y)
        theta2 = math.acos(
            (math.pow(self.armVars['A2'], 2) - math.pow(x, 2) - math.pow(y, 2) + math.pow(self.armVars['A3'], 2)) / (
                    2 * self.armVars['A2'] * self.armVars['A3'])) + theta1 - math.pi
        self.upper_axis.set_setpoint(theta1)
        self.lower_axis.set_setpoint(theta2)

    def ikin(self, pos_vect):
        self.targetPos = pos_vect
        theta1 = math.atan2(pos_vect[0][0], pos_vect[2][0]) + math.acos(
            self.armVars['A1'] / (math.sqrt(math.pow(pos_vect[2][0], 2) + math.pow(pos_vect[1][0], 2))))
        self.shoulder_axis.set_setpoint(theta1)
        L = math.sqrt(math.pow(pos_vect[2][0], 2) + math.pow(pos_vect[0][0], 2) - math.pow(self.armVars['A1'], 2))
        out_vec = np.array([[L], [pos_vect[1][0]], [0]])
        a = self.ikin_2d(out_vec)



    def go_to(self,target_pos, movement_time=.5):
        start_time = time.time()
        elapsed_time = time.time() - start_time
        start_pos = copy(self.pos)
        diff = target_pos - start_pos
        while elapsed_time <= movement_time:
            perc_move = quintic(elapsed_time/movement_time)

            new_target = diff * perc_move + start_pos
            print(new_target)
            try:
                self.ikin(new_target)
            except:
                self.ikin_2d(new_target)

            elapsed_time = time.time()-start_time
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

    def fwkin(self, thetas=None, joint=3, vector=True):
        """
        converts joint angles stored in self.thetas to workspace returns:
        [float]  returns a either a 4x4 matrix of the transform from joint 1 to the input joint or a 3x1 vector
                 depending on the vector variable
        """
        # dh table
        if not thetas:
            thetas = self.thetas

        dh_table = [[0, self.armVars['D1'], self.armVars['A1'], math.pi / 2],
                    [0, 0, self.armVars['A2'], 0],
                    [0, 0, self.armVars['A3'], 0]]
        # identity matrix
        t_final = np.identity(4)
        # calculate fwkin
        for i in range(joint - 1):
            params = dh_table[i]
            dh_table[0] += thetas[i][0]
            t_final *= htm(*params)

        # if vector retun just the pos var
        if vector:
            return t_final[0:3][3]

        return t_final

    def update(self):
        # gets angle from each of the three joints

        t1 = self.shoulder_axis.get_pos()
        t2 = self.upper_axis.get_pos()
        t3 = self.lower_axis.get_pos()

        self.thetas = np.array([[t1], [t2], [t3]])

        self.pos = self.fwkin()

        self.vel = np.array([[self.shoulder_axis.get_vel()], [self.upper_axis.get_vel()], [self.lower_axis.get_vel()]])

        self.accel = np.array(
            [[self.shoulder_axis.get_accel()], [self.upper_axis.get_accel()], [self.lower_axis.get_accel()]])

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
