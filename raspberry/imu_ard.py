#!python3
# Author: Theodor Giles
# Created: 7/15/20
# Last Edited 5/19/21
# Description:
# node for moving around IMU data from the arduino

import time
from imu import IMU
import re

GYRO: int = 0
POSITION: int = 1
YAW: int = 0
PITCH: int = 1
ROLL: int = 2
NORTH: int = 0
EAST: int = 1
DOWN: int = 2


class ArduinoIMU(IMU):

    def __init__(self, serial):

        # read info from vehicle
        self.serial = serial
        self.serial.flushInput()

        # arm vehicle to see position
        # print(self.serial.readline())
        # - Read the actual attitude: Roll, Pitch, and Yaw
        self.UpdateGyro()
        self.StartingGyro = self.Angle
        print('Orientation: ', self.getStartingGyro())

        # - Read the actual position North, East, and Down
        # self.UpdatePosition()
        # self.StartingPosition = self.Position
        # print('Position: ', self.getStartingPosition())

        # - Read the actual depth:
        time.sleep(3)
        print("Starting gyro: ", self.StartingGyro)
        # print("Starting position: ", self.Position)

    # parse gyro object data from wt61p, can then pass to other programs
    def UpdateGyro(self):
        angleFront = self.getAngleFront()
        angleRear = self.getAngleRear()
        self.Angle[0] = (angleFront[0] + angleRear[0])/2
        self.Angle[1] = (angleFront[1] + angleRear[1])/2
        self.Angle[2] = (angleFront[2] + angleRear[2])/2

    # parse position object data from wt61p, can then pass to other programs
    def UpdatePosition(self):
        pass

    def getAngleFront(self):
        self.serial.write("gfa\n".encode('utf-8'))
        data = self.serial.read_until("\n")
        # print("Front angle:", data)
        return parseXYZDataToList(data)

    def getAngleRear(self):
        self.serial.write("gra\n".encode('utf-8'))
        data = self.serial.read_until("\n")
        return parseXYZDataToList(data)

    # end command/vehicle running
    def Terminate(self):
        self.serial.write("STOP")
        self.serial.close()


def parseXYZDataToList(xyz_data):
    i = -1
    xyz = [0.0, 0.0, 0.0]
    for parsed in str(xyz_data).split(':'):
        if 4 > i >= 0:
            xyz[i] = float(parsed)
        i = i + 1
    return xyz

#
# class ArduinoHandler(IMU):
#
#     def __init__(self, serial, numWT61P=0, numBN055=0):
#         self.WT61P_list = []
#         self.BN055_list = []
#         for i in range(numWT61P):
#             self.WT61P_list[i] = WT61P(self.serial, i)
#         for i in range(numBN055):
#             self.BN055_list[i] = BN055(self.serial, i)
