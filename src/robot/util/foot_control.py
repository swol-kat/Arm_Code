import numpy as np
from .kinematics import euler_tm
import matplotlib.pyplot as plt
from math import tau

'''
reference: https://www.mdpi.com/2076-3417/9/7/1508/pdf

writing code to provide curve generation of the swing of a single foot

#TODO
implement spline curve
from mat
'''


def swing_pos(t, step_height=1.5, step_length=3, phi=0):
    """
    :param t: percenetage time in movmenet
    :param phi: angle of step between 0, tau/4
    :param step_height: of single step in
    :param step_length: of single step in
    :return x,y,z point np array
    """
    bp = __gen_bezier_points(step_height, step_length)
    rot = euler_tm(phi, 0, 0)

    bp = [rot @ np.array(p).reshape((3, 1)) for p in bp]  # rotates all points by phi

    bp = [p.reshape(3, 1) for p in bp]

    pos = np.zeros(3, 1)
    n = len(bp)
    for i, p in enumerate(bp):
        pos += (1 - t) ** (n - i) * (t ** i) * p

    return pos


def __gen_bezier_points(step_height, step_length):
    """
        generating the 12 bezier points based on what was found in the paper
        reasons to use a bezier curve
        - a set of 2 points ensures zero velocity at that point
        - a set of 3 points ensures zero acceleration at that point
            :param step_height: of single step
            :param step_length: of single step
            :return list of bezier points
    """

    return [
        [0, 0, 0],  # foot starting pos
        [-step_length * .05, 0, 0],
        [-step_length * .1, 0, step_height * 1.1],
        [-step_length * .1, 0, step_height * 1.1],
        [-step_length * .1, 0, step_height * 1.1],
        [step_length / 2, 0, step_height * 1.1],
        [step_length / 2, 0, step_height * .9],
        [step_length / 2, 0, step_height * .9],
        [step_length * 1.1, 0, step_height * 1.1],
        [step_length * 1.1, 0, step_height * 1.1],
        [step_length * 1.05, 0, 0],
        [step_length, 0, 0]
    ]


if __name__ == "__main__":
    test = [
        (1.5,3, 0), (1.5,3, tau/4) , (1.5,3, tau/2)
    ]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    fig.show()

    ax.set_xlim([-2, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([0, 10])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legent()

    for step_height, step_length, phi in test:
        for i in np.linspace(0,1):
            x,y,z = swing_pos(i, step_height, step_length, phi)
            ax.plot(x, y, z, '-', label=f'sh:{step_height}, sl:{step_length}, phi:{phi} ')



