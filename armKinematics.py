import robotJoint
import math
import numpy as np
from kinematics import htm
from enum import Enum

class ArmVariables(Enum):
    A1 = 0
    D1 = 1
    A2 = 2
    A3 = 3


class Arm:
    def __init__(self, lowerAxis, upperAxis, shoulderAxis, armVars):
        self.lowerAxis = lowerAxis
        self.upperAxis = upperAxis
        self.shoulderAxis = shoulderAxis
        self.armVars = armVars

        self.targetPos = np.array([[0],[0],[0]])
        self.thetas = np.array([[0],[0],[0]])
        self.pos = np.array([[0],[0],[0]])
        self.vel = np.array([[0],[0],[0]])
        self.accel = np.array([[0],[0],[0]])
        self.torque = np.array([[0],[0],[0]])

    def goTo2dPos(self, posVect):
        x = posVect[0,0]
        y = posVect[0,1]
        theta1 = math.acos((math.pow(self.armVars[armVariables.A2], 2) + math.pow(x, 2) + math.pow(y, 2) - math.pow(self.armVars[armVariables.A3], 2))/(2 * self.armVars[armVariables.A2] * math.sqrt(math.pow(x, 2) + math.pow(y, 2)))) - math.atan2(x,y)
        theta2 = math.acos((math.pow(self.armVars[armVariables.A2], 2) - math.pow(x, 2) - math.pow(y, 2) + math.pow(self.armVars[armVariables.A3], 2))/(2 * self.armVars[armVariables.A2] * self.armVars[armVariables.A3])) + theta1 - math.pi
        self.upperAxis.setSetpoint(theta1)
        self.lowerAxis.setSetpoint(theta2)
    
    def goToPosition(self, posVect):
        self.targetPos = posVect
    
    def setCurrentLimits(self, minForce, maxForce):
        #know the jacobian and maths
        # F_tip = T * J(q)
        pass
    

    def jabobian(self, thetas = None):
        if not thetas:
            thetas = self.thetas

        pe = self.fwkin(thetas)

        J = np.zeros(6,1)
        for i in range(3):
            tf = self.fwkin(thetas, joint = i, vector=False)
            z = tf[0:3,i]
            pi = tf[0:3,3]

            Jp = np.cross(z, pe - pi)
            Jo = z
            Ji = np.concatenate((Jp,Jo),axis=0)
            J = np.concatenate((J,Ji),axis=1)

        return np.delete(J,0,1)

    def fwkin(self, thetas = None, joint = 3 , vector = True):
        '''
        converts joint angles stored in self.thetas to workspace
        returns:
        [float]  returns a either a 4x4 matrix of the transform from joint 1 to the input joint or a 3x1 vector depending on the vector variable
        '''
        # dh table
        if not thetas: 
            thetas = self.thetas
        dh_table = [['t','d','a','alp'],
                    ['t','d','a','alp'],
                    ['t','d','a','alp']]
        # identity matrix
        t_final = np.identity(4)
        # calculate fwkin
        for i in range(joint-1):
            params = dh_table[i]
            dh_table[0] += thetas[0,i]
            t_final *= htm(*params)
    
        # if vector retun just the pos var
        if vector:
            return t_final[0:3,3]

        return t_final

    def update(self):
        #gets angle from each of the three joints

        t1 = self.shoulderAxis.get_pos()
        t2 = self.upperAxis.get_pos()
        t3 = self.lowerAxis.get_pos()

        self.thetas = np.array([[t1],[t2],[t3]])

        self.pos = self.fwkin()
    
        self.vel = np.array([[self.shoulderAxis.get_vel()],[self.upperAxis.get_vel()],[self.lowerAxis.get_vel()]])

        self.accel = np.array([[self.shoulderAxis.get_accel()],[self.upperAxis.get_accel()],[self.lowerAxis.get_accel()]])

        self.torque = np.array([[self.shoulderAxis.get_torque()],[self.upperAxis.get_torque()],[self.lowerAxis.get_torque()]])
    