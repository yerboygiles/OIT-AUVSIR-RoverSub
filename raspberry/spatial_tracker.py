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
        # x, y, z
        self.location = [0.0, 0.0, 0.0]
        # x, y, z, ID, 3D map
        self.hardpoints = [0.0, 0.0, 0.0, "ID placeholder", []]
        self.targets = [[0.0, 0.0, 0.0, "ID placeholder"], []]

    def translateSelf(self, x, y, z):
        self.location[0] += x
        self.location[1] += y
        self.location[2] += z

    def addHardpoint(self, x, y, z, id="unknown"):
        self.hardpoints.append([x, y, z, id])

    def modifyHardpoint(self, x, y, z, id):
        hardpindex = 0
        if isinstance(id, str):
            i = 0
            for hardpoint in self.hardpoints:
                if id is hardpoint[3]:
                    hardpindex = i
                    break
                i = i + 1
        else:
            hardpindex = id
        self.hardpoints[hardpindex][0] = x
        self.hardpoints[hardpindex][1] = y
        self.hardpoints[hardpindex][2] = z

    def translateHardpoint(self, x, y, z, id="unknown"):
        hardpindex = 0
        if isinstance(id, str):
            i = 0
            for hardpoint in self.hardpoints:
                if id is hardpoint[3]:
                    hardpindex = i
                    break
                i = i + 1
        else:
            hardpindex = id
        self.hardpoints[hardpindex][0] += x
        self.hardpoints[hardpindex][1] += y
        self.hardpoints[hardpindex][2] += z

    def addTarget(self, x, y, z, identifier="unknown"):
        self.targets.append([x, y, z, identifier])

    def modifyTarget(self, x, y, z, id):
        targindex = 0
        if isinstance(id, str):
            i = 0
            for target in self.targets:
                if id is target[3]:
                    targindex = i
                    break
                i = i + 1
        else:
            targindex = id
        self.targets[targindex][0] = x
        self.targets[targindex][1] = y
        self.targets[targindex][2] = z

    def translateTarget(self, x, y, z, id):
        targindex = 0
        if isinstance(id, str):
            i = 0
            for target in self.targets:
                if id is target[3]:
                    targindex = i
                    break
                i = i + 1
        else:
            targindex = id
        self.targets[targindex][0] += x
        self.targets[targindex][1] += y
        self.targets[targindex][2] += z

    def getTargetVector(self, id):
        pass
