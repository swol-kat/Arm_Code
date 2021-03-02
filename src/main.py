from robot import Robot
from math import tau
import numpy as np

r = Robot()

r.boot()


for i in np.linspace(0,tau/12,10):
    r.target_base_state.gamma = i/4
    r.loop()

for i in np.linspace(tau/12, -tau/12,20):
    r.target_base_state.gamma = i/4
    r.loop()

for i in np.linspace(-tau/12, 0,10):
    r.target_base_state.gamma = i/4
    r.loop()

input('brr')