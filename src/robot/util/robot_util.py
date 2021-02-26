from .bodystate import BodyState
import numpy as np
from .kinematics import euler_tm
from math import tau


def get_body_pts(state: BodyState, width: float, length: float):
    """
    :param width: robot width
    :param length: robot length
    :param state: state to get info about
    :return:
    """
    cg = np.array([state.x, state.y, state.z])

    body_pts = [[width/2, length/2, 0], [-width/2, length/2, 0], [-width/2, -length/2, 0], [width/2, -length/2, 0]]
    # convert to np array
    body_pts = [np.array(pt) for pt in body_pts]
    # calculate rot matrix for body
    rot = euler_tm(state.alpha, state.beta, state.gamma)
    # multiply by rotation
    body_pts = [rot @ pt.reshape((3, 1)) for pt in body_pts]
    # add x,y,z
    body_pts = [cg + pt.reshape(3) for pt in body_pts]

    return body_pts


def get_rot_leg_orig(leg_num ):
    rot_leg_orig = [euler_tm(0, tau / 4, 0), euler_tm(tau / 4, tau / 4, 0), euler_tm(tau / 2, tau / 4, 0),
                    euler_tm(-tau / 4, tau / 4, 0)]



    return rot_leg_orig[leg_num]
