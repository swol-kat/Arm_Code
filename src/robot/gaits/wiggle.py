import numpy as np

from .gait import Gait
from ..util import get_body_pts, get_rot_leg_orig


class Wiggle(Gait):

    def __init__(self):
        super().__init__()

    def loop(self, robot):
        """
        :param self.parms contains zcg and target orientation for this gait
        :param robot: takes in the robot object
        :return:
        """

        width = robot.config["width"]
        length = robot.config["length"]

        body_pts = get_body_pts(robot.target_base_state, width, length)
        for i, leg in enumerate(robot.arms):
            # get_taget_foot_pos in world points
            body_pts[i][2] = 0
            print(body_pts[i])
            target_foot_pos = body_pts[i].reshape((3,1))
            rot_orig_leg = get_rot_leg_orig(i)
            leg.target_pos = rot_orig_leg @ target_foot_pos
