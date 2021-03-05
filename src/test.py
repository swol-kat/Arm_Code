from robot.util import swing_pos
import matplotlib.pyplot as plt
import numpy as np
from math import tau

if __name__ == "__main__":
    test = [
        (1.5, 3, 0), (1.5, 3, tau / 8), (1.5, 3, tau / 4), (2, 6, 0), (2, 3, 0), (1,4,0)
    ]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    fig.show()

    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([0, 5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    for step_height, step_length, phi in test:
        p = []
        for i in np.linspace(0, 1):
            p.append(swing_pos(i, step_height, step_length, phi))
        p = np.array(p)
        ax.plot(p[:, 0], p[:, 1], p[:, 2], '-', label=f'sh:{step_height}, sl:{step_length}, phi:{phi} ')

    ax.legend()

    input('brr')
