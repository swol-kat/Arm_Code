import math
import numpy as np

import time
from .util.kinematics import htm
from .util import quintic
from .util import Plot
from copy import copy, deepcopy

class Arm:
    def __init__(self, lower_axis, upper_axis, shoulder_axis, arm_vars, plot=False):
        self.lower_axis = lower_axis
        self.upper_axis = upper_axis
        self.shoulder_axis = shoulder_axis
        self.arm_vars = arm_vars

        self.target_pos = np.array([[0], [0], [0]])
        self.thetas = np.array([[0], [0], [0]])
        self.pos = np.array([[0.], [0.], [0.]])
        self.joint_vel = np.array([[0], [0], [0]])
        self.joint_torque = np.array([[0], [0], [0]])
        self.tip_force_limit = np.array([[3],[3],[3]])
        self.vel = np.zeros((3,1))
        self.force = np.zeros((3,1))

        self.plot=False
        if plot:
            self.plot = Plot()


        self.update()

    def ikin(self, pos_vect, dog=True):
        x, y, z = pos_vect.reshape(3)
        d1, d2, a2, a3 = self.arm_vars.values()
        # t1
        r = math.hypot(x, y)
        u = math.sqrt(r ** 2 - d2 ** 2)
        alpha = math.atan2(y, x)
        beta = math.atan2(d2, u)

        t1 = alpha - beta

        s = z - d1
        l = math.hypot(s, u)
        D = (l ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
        # making sure D doesnt excced -1 one cause floating points
        D = min(max(D, -1), 1)
        # t2
        if dog:
            t3 = - math.atan2(math.sqrt(1 - D ** 2), D)
        else:
            t3 = - math.atan2(- math.sqrt(1 - D ** 2), D)
        phi = math.atan2(s, u)
        gamma = math.atan2(a3 * math.sin(t3), a2 + a3 * math.cos(t3))
        t2 = - (gamma + phi)
        t3 = t2 + t3
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

        pe = self.fwkin(thetas).reshape((1,3))

        J = np.zeros((6, 1))
        for i in range(3):
            tf = self.fwkin(thetas, joint=i, vector=False)
            z = tf[0:3, 2]
            pi = tf[0:3, 3]
            Jp = np.cross(z, pe - pi)
            Jo = z
            Ji = np.concatenate((Jp.reshape((3,1)), Jo.reshape((3,1))), axis=0)
            J = np.concatenate((J, Ji), axis=1)

        return np.delete(J, 0, 1)

    def fwkin(self, thetas=None, joint=3, vector=True, disp = False):
        """
        converts joint angles stored in self.thetas to workspace returns:
        [float]  returns a either a 4x4 matrix of the transform from joint 1 to the input joint or a 3x1 vector
                 depending on the vector variable
        """
        # dh table
        if thetas is None:
            thetas = self.thetas

        t1, t2, t3 = thetas.reshape(3)
        if disp:
            dh_table = [[t1, self.arm_vars['D1'], 0, - math.pi / 2],
                        [0, self.arm_vars['D2'], 0, 0],
                        [t2, 0, self.arm_vars['A2'], 0],
                        [t3-t2, 0, self.arm_vars['A3'], 0]]

        else:
            dh_table = [[t1, self.arm_vars['D1'], 0, - math.pi / 2],
                        [t2, self.arm_vars['D2'], self.arm_vars['A2'], 0],
                        [t3-t2, 0, self.arm_vars['A3'], 0]]
        # identity matrix
        t_final = np.identity(4)
        # calculate fwkin
        for i in range(joint):
            params = dh_table[i]
            t_final = t_final @ htm(*params)
        # if vector return just the pos var
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

        self.joint_vel = np.array([[self.shoulder_axis.get_vel()], [self.upper_axis.get_vel()], [self.lower_axis.get_vel()]])

        self.joint_torque = np.array([[self.shoulder_axis.get_torque()], [self.upper_axis.get_torque()], [self.lower_axis.get_torque()]])

        self.vel = self.get_tip_vel()

        self.force= self.get_tip_force()

        if self.plot:
            self.plot.plot(self)

        # do tip forces
        """
        curr_force = self.get_tip_force()
        delta = self.tip_force_limit - np.abs(curr_force)
        torque_delta = np.transpose(self.jacobian()[0:3,:]) @ delta
        self.shoulder_axis.set_torque(np.abs(self.shoulder_axis.get_torque()) + torque_delta[0])
        self.upper_axis.set_torque(np.abs(self.upper_axis.get_torque()) + torque_delta[1])
        self.lower_axis.set_torque(np.abs(self.lower_axis.get_torque()) + torque_delta[2])
        """

    def home_arm(self):
        print("homing shoudler")
        self.shoulder_axis.run_manual_homing_routine()
        print("homing upper")
        self.upper_axis.run_manual_homing_routine()
        print("homing lower")
        self.lower_axis.run_manual_homing_routine()

    def calibrate_arm(self):
        print("calibrating arm")
        self.shoulder_axis.start_calibration()
        self.upper_axis.start_calibration()
        self.lower_axis.start_calibration()
        end = False
        while not end:
            end = True
            end = end and self.shoulder_axis.is_calibration_complete()
            end = end and self.upper_axis.is_calibration_complete()
            end = end and self.lower_axis.is_calibration_complete()

    def enable_arm(self):
        print("enabling arm")
        self.shoulder_axis.enable_joint()
        self.upper_axis.enable_joint()
        self.lower_axis.enable_joint()

    def fuck(self):
        print("fucking arm")
        self.shoulder_axis.fuck()
        self.upper_axis.fuck()
        self.lower_axis.fuck()


    def get_joint_pos(self):
        xs = [0]
        ys = [0]
        zs = [0]
        for i in range(4):
            pos = self.fwkin(joint=i + 1, disp=True)
            x, y, z = pos.reshape(3)
            xs.append(x)
            ys.append(y)
            zs.append(z)
        return {
            'x': xs,
            'y': ys,
            'z': zs
        }

    def jog(self,thetas,pos):
        if thetas and np.sum(thetas) != 0:
            thetas = np.array(thetas).reshape((3,1))
            self.send_to_pos(self.thetas + thetas)
        if pos and np.sum(pos) != 0:
            pos = np.array(pos).reshape((3, 1))
            self.go_to_raw(self.pos+pos,False)

    def export_data(self):
        joint_pos = self.get_joint_pos()
        forces = deepcopy(self.vel.reshape(3))
        joint_pos['x'].append(forces[0] + joint_pos['x'][-1])
        joint_pos['y'].append(forces[1] + joint_pos['y'][-1])
        joint_pos['z'].append(forces[2] + joint_pos['z'][-1])

        print(self.jacobian())
        print('-----------------------------------')

        return {
            'joint_pos': joint_pos,
            'thetas': self.thetas.reshape(3).tolist(),
            'pos': self.pos.reshape(3).tolist(),
            'joint_vel': self.joint_vel.reshape(3).tolist(),
            'joint_torque': self.joint_torque.reshape(3).tolist(),
            'vel': self.vel.reshape(3).tolist(),
            'force': self.force.reshape(3).tolist(),
        }

    def get_error(self):
        return {
            'shoulder': self.shoulder_axis.get_error(),
            'upper': self.upper_axis.get_error(),
            'lower': self.lower_axis.get_error()
        }

    def get_tip_vel(self):
        jacob = self.jacobian()[0:3,:]

        tip_vel = jacob @ self.joint_vel

        return tip_vel.reshape((3,1))

    def get_tip_force(self):
        jacob = self.jacobian()[0:3,:]

        tip_force = jacob @ self.joint_torque

        return tip_force.reshape((3,1))/100

    def set_tip_force_limit(self, x, y, z):
        self.tip_force_limit = np.array([[x],[y],[z]])
