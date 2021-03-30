import math
import time

import numpy as np

from .gait import Gait
from ..util import get_body_pts, get_rot_leg_orig, euler_tm, swing_pos, ground_pos


class OpenWalk(Gait):

    def __init__(self):
        super().__init__()
        self.params = {
            'step_time': .5,  # seconds per movment
            'step_height': 2,  # inches,
        }
        self.last_loop_time = time.time()
        self.last_time = time.time()
        self.state = [1, 3]
        self.movement_vector = {}
        self.x_vel = 0
        self.y_vel = 0
        self.count = 0
        self.prev_foot_pos = [np.zeros(3) for i in range(4)];

    def loop(self, robot):
        width = robot.config["width"]
        length = robot.config["length"]
        step_time = self.params['step_time']

        delta_t = (time.time() - self.last_loop_time)


        body_pts = get_body_pts(robot.target_base_state, width, length, False)

        rot = euler_tm(robot.target_base_state.alpha, robot.target_base_state.beta, robot.target_base_state.gamma)
        l = math.hypot(self.x_vel, self.y_vel) * step_time
        phi = math.atan2(self.y_vel, self.x_vel)
        d_swing = swing_pos(min(delta_t / step_time, 1), self.params['step_height'], l, phi)
        d_ground = ground_pos(min(delta_t / step_time, 1), l, phi)
        for i, leg in enumerate(robot.arms):
            target_foot_pos = np.array([body_pts[i][0], body_pts[i][1], 0]) + self.prev_foot_pos[i]
            if i in self.state:
                target_foot_pos += d_swing
            else:
                target_foot_pos += d_ground

            target_foot_pos = target_foot_pos - body_pts[i]
            if delta_t > step_time:
                self.prev_foot_pos[i] = np.append(target_foot_pos[:2],0)
            target_foot_pos = get_rot_leg_orig(i).transpose() @ rot.transpose() @ target_foot_pos
            leg.target_pos = target_foot_pos

        
        if delta_t > step_time:
            self.last_loop_time = time.time()
 
            self.next_state()
            if self.count == 1:
                self.movement_vector = robot.movement_vector
                self.x_vel = robot.movement_vector['x']
                self.y_vel = robot.movement_vector['y']

            self.count = (self.count + 1) % 2

    def next_state(self):
        for i, leg in enumerate(self.state):
            self.state[i] += 1
            self.state[i] %= 4
