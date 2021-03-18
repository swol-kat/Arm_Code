import json

from .arm import Arm
from .virtual_joint import VirtualJoint
from .gaits import Gait, Wiggle, OpenWalk
from .util import BodyState, Plot


class Robot:
    config: dict
    gait: Gait
    arms: list
    base_state: BodyState
    target_base_state: BodyState
    movement_vector: dict

    def __init__(self):
        self.reload_config()
        self.plot = Plot()

    def boot(self):
        """
        boot up routine for robot
        :return:
        """
        self.arms = [
            Arm(VirtualJoint(), VirtualJoint(), VirtualJoint(), self.config['arm_data'], corner=1),
            Arm(VirtualJoint(), VirtualJoint(), VirtualJoint(), self.config['arm_data'], corner=2),
            Arm(VirtualJoint(), VirtualJoint(), VirtualJoint(), self.config['arm_data'], corner=3),
            Arm(VirtualJoint(), VirtualJoint(), VirtualJoint(), self.config['arm_data'], corner=4)
        ]

        for arm in self.arms:
            arm.calibrate_arm()
            arm.home_arm()
            arm.home_arm()

        self.base_state = BodyState(z=11)
        self.target_base_state = BodyState(z=11)
        self.movement_vector = dict(x=0, y=0, z=0, alpha=0, beta=0, gamma=0)

        self.gait = OpenWalk()

    def loop(self):
        """
        main control loop of robot run this in a while loop or something
        :return:
        """
        self.base_state = self.target_base_state
        for arm in self.arms:
            arm.update()
        self.plot.plot(self)
        if self.gait:
            self.gait.loop(self)
        for arm in self.arms:
            arm.loop()

    def reload_config(self):
        """
        reloads the robot_config file
        :return:
        """
        self.config = json.load(open('./robot/robot_config.json'))


if __name__ == "__main__":
    robot = Robot()
