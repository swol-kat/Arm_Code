import numpy as np


def s(angle):
    return np.sin(angle)


def c(angle):
    return np.cos(angle)


def htm(theta, d, a, alpha):
    return np.array([[c(theta), -1.0 * s(theta) * c(alpha), s(theta) * s(alpha), a * c(theta)],
                     [s(theta), c(theta) * c(alpha), -1.0 *c(theta) * s(alpha), a * s(theta)],
                     [0, s(alpha), c(alpha), d],
                     [0, 0, 0, 1]])