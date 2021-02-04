import robotJoint
import math
import numpy as np
from kinematics import htm
from enum import Enum


class Arm:
    def __init__(self, lowerAxis, upperAxis, shoulderAxis, armVars):
        self.lowerAxis = lowerAxis
        self.upperAxis = upperAxis
        self.shoulderAxis = shoulderAxis
        self.armVars = armVars

        self.targetPos = np.array([[0],[0],[0]])
        self.thetas = np.array([[0],[0],[0]]) #
        self.pos = np.array([[0],[0],[0]])
        self.vel = np.array([[0],[0],[0]])
        self.accel = np.array([[0],[0],[0]])
        self.torque = np.array([[0],[0],[0]])

    def goTo2dPos(self, posVect):
        x = posVect[0][0]
        y = posVect[1][0]
        theta1 = math.acos((math.pow(self.armVars['A2'], 2) + math.pow(x, 2) + math.pow(y, 2) - math.pow(self.armVars['A3'], 2))/(2 * self.armVars['A2'] * math.sqrt(math.pow(x, 2) + math.pow(y, 2)))) - math.atan2(x,y)
        theta2 = math.acos((math.pow(self.armVars['A2'], 2) - math.pow(x, 2) - math.pow(y, 2) + math.pow(self.armVars['A3'], 2))/(2 * self.armVars['A2'] * self.armVars['A3'])) + theta1 - math.pi
        self.upperAxis.setSetpoint(theta1)
        self.lowerAxis.setSetpoint(theta2)
    
    def goToPosition(self, posVect):
        self.targetPos = posVect
        theta1 = math.atan2(posVect[0][0], posVect[2][0]) + math.acos(self.armVars['A1']/(math.sqrt(math.pow(posVect[2][0], 2) + math.pow(posVect[1][0], 2))))
        self.shoulderAxis.setSetpoint(theta1)
        L = math.sqrt(math.pow(posVect[2][0], 2) + math.pow(posVect[0][0], 2) - math.pow(self.armVars['A1'], 2))
        outVect = np.array([[L], [posVect[1][0]], [0]])
        self.goTo2dPos(outVect)
    
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
            
        dh_table = [[0 , self.armVars['D1'], self.armVars['A1'] , math.pi/2],
                    [0 , 0 , self.armVars['A2'], 0],
                    [0 , 0 , self.armVars['A3'], 0]]
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

    def homeArm(self):
        print("homing shoudler")
        self.shoulderAxis.runManualHomingRoutine()
        print("homing upper")
        self.upperAxis.runManualHomingRoutine()
        print("homing lower")
        self.lowerAxis.runManualHomingRoutine()
    
    def calibrateArm(self):
        self.shoulderAxis.calibrateJoint()
        self.upperAxis.calibrateJoint()
        self.lowerAxis.calibrateJoint()

    def enableArm(self):
        self.shoulderAxis.enableJoint()
        self.upperAxis.enableJoint()
        self.lowerAxis.enableJoint()
    
    