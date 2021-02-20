from math import tau

import matplotlib.pyplot as plt
import numpy as np

from .kinematics import euler_tm


class Plot:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.fig.show()
        self.width = 20.
        self.length = 30.

    def config_plt(self):
        # self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-50, 50])
        self.ax.set_ylim([-50, 50])
        self.ax.set_zlim([0, 30])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

    def plot(self, base_state: np.array, arms: list):
        """
        :param arms: list
        :type base_state: np.array
        takes in base state and arms param and plots them
        :return:
        """
        xcg, ycg, zcg, psi, theta, phi = base_state.reshape(6)

        # clearing and configing plot
        self.ax.clear()
        self.config_plt()
        # mark center point
        self.ax.plot(0, 0, 0, marker='o', markersize=3, color="red")
        self.ax.plot(0, 0, zcg, marker='o', markersize=3, color="blue")
        # calculate body points from body
        body_x = [self.width, -self.width, -self.width, self.width, self.width]
        body_y = [self.length, self.length, -self.length, -self.length, self.length]
        body_z = [0, 0, 0, 0, 0]

        rot = euler_tm(phi, theta, psi)
        # rotate body points by euler angle
        for i in range(len(body_x)):
            point = np.array([body_x[i], body_y[i], body_z[i]]).reshape(3, 1)
            new_point = rot @ point
            body_x[i], body_y[i], body_z[i] = new_point.reshape(3)
            body_z[i] += zcg

        self.ax.plot(body_x, body_y, body_z, 'o-')

        # plotting legs points
        rot_orig_leg = [euler_tm(0, tau / 4, 0), euler_tm(tau / 4, tau / 4, 0), euler_tm(tau / 2, tau / 4, 0),
                        euler_tm(-tau / 4, tau / 4, 0)]

        for i, leg in enumerate(arms):

            rotated_pts = rot @ rot_orig_leg[i] @ leg.pos.reshape(3, 1)
            body_pts = np.array([body_x[i], body_y[i], body_z[i]])
            x, y, z = rotated_pts.reshape(3) + body_pts

            color = 'red'
            if leg.contact:
                color = 'green'

            # calculate leg pos
            xs, ys, zs = [[p] for p in body_pts.reshape(3)]
            rot_leg = rot_orig_leg[i]
            for i in range(4):
                pos = leg.fwkin(joint=i + 1, disp=True)
                pos = rot @ rot_leg @ pos
                xy, yy, zy = pos.reshape(3) + body_pts
                xs.append(xy)
                ys.append(yy)
                zs.append(zy)

            self.ax.plot(xs, ys, zs, 'o-')

            # convert from foot space to world space
            self.ax.plot(x, y, z, marker='o', markersize=5, color=color)
        self.fig.canvas.draw()


if __name__ == "__main__":
    p = Plot()
    p.plot()
