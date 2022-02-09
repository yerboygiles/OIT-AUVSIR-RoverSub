#!python3
# Author: Theodor Giles
# Created: 11/5/21
# Last Edited 11/5/21
# Description:
# This node manages the libraries of hardpoint locations
# and other varying locations relating to the roversub
# that may need future referencing for navigation,
# self-repair,


# from threading import Thread

# ROBOSUB
ARM_LEFT = [0.0, 0.0, 0.0]
ARM_RIGHT = [0.0, 0.0, 0.0]

STEREO_CAM_LEFT = [0.0, 0.0, 0.0]
STEREO_CAM_RIGHT = [0.0, 0.0, 0.0]


class SpatialTracker:
    # initialize everything to supposed starting position
    def __init__(self):
        self.location = [0.0, 0.0, 0.0]
        self.targets = [[0.0, 0.0, 0.0, "idplaceholder"]]

    def translateTarget(self, id, x, y, z):
        targindex = 0
        if isinstance(id, str):
            i = 0
            for target in self.targets:
                if id is target[3]:
                    targindex = i
                i = i + 1
        else:
            targindex = id
        self.targets[targindex][0] += x
        self.targets[targindex][1] += y
        self.targets[targindex][2] += z

    def setTarget(self, id, x, y, z):
        targindex = 0
        if isinstance(id, str):
            i = 0
            for target in self.targets:
                if id is target[3]:
                    targindex = i
                i = i + 1
        else:
            targindex = id
        self.targets[targindex][0] = x
        self.targets[targindex][1] = y
        self.targets[targindex][2] = z

    def translateLocation(self, x, y, z):
        self.location[0] += x
        self.location[1] += y
        self.location[2] += z

    def addTarget(self, x, y, z, identifier="unknown"):
        self.targets.append([x, y, z, identifier])

    def getTargetVector(self, id):
        pass