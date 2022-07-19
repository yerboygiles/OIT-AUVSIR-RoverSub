#!python3
# Author: Theodor Giles
# Created: 7/15/21
# Last Edited 7/16/21
# Description:
# testing script for vision system

import vision_v1

print("Running vision without full sys. ")


def runwithoutfullsys():
    # 3, 1, 2 - running on laptop
    # 2, 0, 1 - running on pi with only stereo setup
    # 3, 0, 1 - running on pi with stereo + down facing cam
    Vision = vision_v1.vision(3, 1, 2)
    while True:
        Vision.StereoTarget(True)
        print("X Offset: ", Vision.getXOffset())
        print("Y Offset: ", Vision.getYOffset())


runwithoutfullsys()
print("Terminating...")
