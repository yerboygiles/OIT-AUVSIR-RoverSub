#!python3
# Author: Theodor Giles
# Created: 11/5/21
# Last Edited 11/5/21
# Description:
# This node manages the libraries of hardpoint locations
# and other varying locations related to the platform that
# need referencing

import time
import random
import math
import serial

# from threading import Thread

# ROBOSUB
ARM_LEFT = [0.0, 0.0, 0.0]
ARM_RIGHT = [0.0, 0.0, 0.0]

STEREO_CAM_LEFT = [0.0, 0.0, 0.0]
STEREO_CAM_RIGHT = [0.0, 0.0, 0.0]


class SelfAwareness:

    # initialize everything to supposed starting position
    def __init__(self, usingvision=False, usinggyro=False, usingsim=False, resetheadingoncmd=False):