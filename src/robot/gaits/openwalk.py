from .gait import Gait
from ..util import get_body_pts, get_rot_leg_orig, euler_tm
from ..util import BodyState
import time


class OpenWalk(Gait):

    def __init__(self):
        super().__init__()
        self.params = {
            'x_vel': 1,  # forward velocity of quadruped in/s ?
            'cycle time': 5,  # seconds for full cycle aka movement of 4 legs
            'step height': 2,  # inches
        }

    def loop(self, robot):
        print(robot.movement_vector)

