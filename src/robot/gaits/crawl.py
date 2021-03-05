from .gait import Gait
from ..util import get_body_pts, get_rot_leg_orig, euler_tm
from ..util import BodyState
import time


class Crawl(Gait):

    def __init__(self):
        super().__init__()
        self.params = {
            'x_vel': 1,  # forward velocity of quadruped in/s ?
            'cycle time': 5,  # seconds for full cycle aka movement of 4 legs
            'step height': 2,  # inches
        }
        self.gait_state = {
            'phase': 0,  # describes which 3 legs the triangle is made with
            'phase_start_time': time.time()

        }

    def loop(self, Robot):
        pass

    def __next_phase(self):
        self.gait_state['phase'] += 1  # increments +1
        self.gait_state['phase'] %= 4  # mods so if >4 back to zero
        self.gait_state['phase_start_time'] = time.time()  # reset time since last phase change
