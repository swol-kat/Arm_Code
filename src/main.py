from robot import Robot
from math import tau
import numpy as np

r = Robot()

r.boot()

for i in np.linspace(0,tau/12):
    r.target_base_state.gamma = i
    r.target_base_state.alpha = -i / 2
    r.loop()

for i in np.linspace(tau/12,-tau/12):
    r.target_base_state.gamma = i
    r.target_base_state.alpha = -i/2
    r.loop()

input('brr')