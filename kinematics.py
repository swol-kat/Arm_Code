import numpy as np

def htm(theta, d , a, alpha):
    return np.array([[c(theta), -s(theta)*c(alpha), s(theta)*s(alpha) , a*c(theta)],
                     [s(theta), c(theta)*c(alpha), -c(theta)*s(alpha), a*s(theta)],
                     [0, s(alpha), c(alpha), d],
                     [0, 0, 0 , 1]])
    

def s(angle):
    return np.sin(angle)

def c(angle):
    return np.cos(angle)