from robot import Robot
from math import tau
import numpy as np
import keyboard

r = Robot()
r.boot()
movement_amount =2  # in/s
key_dict = {
    'w': {'axis': 'y', 'dir': 1.},
    's': {'axis': 'y', 'dir': -1.},
    'a': {'axis': 'x', 'dir': -1.},
    'd': {'axis': 'x', 'dir': 1.},
    'up': {'axis': 'z', 'dir': 1.},
    'down': {'axis': 'z', 'dir': -1.},
    'i': {'axis': 'gamma', 'dir': -.1},
    'k': {'axis': 'gamma', 'dir': .1},
    'l': {'axis': 'beta', 'dir': .1},
    'j': {'axis': 'beta', 'dir': -.1},
    'right': {'axis': 'alpha', 'dir': -.1},
    'left': {'axis': 'alpha', 'dir': .1},
}

while True:
    movement_dict = {'x': 0, 'y': 0,'z':0, 'alpha':0,'beta':0,'gamma':0}

    for key in key_dict.keys():
        if keyboard.is_pressed(key):
            if movement_dict[key_dict[key]['axis']] != 0:
                movement_dict[key_dict[key]['axis']] = 0
            else:
                movement_dict[key_dict[key]['axis']] = movement_amount * key_dict[key]['dir']

    r.movement_vector = movement_dict
    r.loop()
