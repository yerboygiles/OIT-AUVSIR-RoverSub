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
    StartingFrontAngle = [0.0, 0.0, 0.0]
    StartingRearAngle = [0.0, 0.0, 0.0]
    FrontAngle = [0.0, 0.0, 0.0]
    RearAngle = [0.0, 0.0, 0.0]

    def __init__(self, serial):
        # read info from vehicle
        self.serial = serial
        self.serial.flushInput()

        # arm vehicle to see position
        # print(self.serial.readline())
        # - Read the actual attitude: Roll, Pitch, and Yaw
        # time.sleep(3)
        self.CalibrateStart()
        print("Starting Front Angle: ", self.StartingFrontAngle)
        print("Starting Rear Angle: ", self.StartingRearAngle)
        # print('Orientation: ', self.getStartingAngle())

        # - Read the actual position North, East, and Down
        # self.UpdatePosition()
        # self.StartingPosition = self.Position
        # print('Position: ', self.getStartingPosition())

        # - Read the actual depth:
        time.sleep(3)
        # print("Starting position: ", self.Position)

    # parse gyro object data from wt61p, can then pass to other programs
    def UpdateAngle(self):
        angleFront = self.getAngleFront()
        angleRear = self.getAngleRear()
        startFront = self.getStartingFrontAngle()
        startRear = self.getStartingRearAngle()
        self.Angle[0] = ((angleFront[0]-startFront[0]) + (angleRear[0]-startRear[0])) / 2
        self.Angle[1] = ((angleFront[1]-startFront[1]) - (angleRear[1]-startRear[1])) / 2
        self.Angle[2] = ((angleFront[2]-startFront[2]) - (angleRear[2]-startRear[2])) / 2

    def CalibrateStart(self):
        angle = self.getAngleFront()
        self.StartingFrontAngle = angle
        angle = self.getAngleRear()
        self.StartingRearAngle = angle

    def UpdateFrontAngle(self):
        angleFront = self.getAngleFront()
        self.Angle[0] = (angleFront[0])
        self.Angle[1] = (angleFront[1])
        self.Angle[2] = (angleFront[2])
        return self.Angle
        # print("Front Angle: ", self.Angle)

    def UpdateRearAngle(self):
        angleRear = self.getAngleRear()
        self.Angle[0] = (angleRear[0])
        self.Angle[1] = (angleRear[1])
        self.Angle[2] = (angleRear[2])
        return self.Angle
        # print("Rear Angle: ", self.Angle)

    # parse position object data from wt61p, can then pass to other programs
    def UpdatePosition(self):
        pass

    def getAngleFront(self):
        self.serial.write("gfa\n".encode('utf-8'))
        data = self.serial.read_until("\n")
        time.sleep(0.05)
        return parseXYZDataToList(data)

    def getAngleRear(self):
        self.serial.write("gra\n".encode('utf-8'))
        data = self.serial.read_until("\n")
        time.sleep(0.05)
        return parseXYZDataToList(data)

    def getStartingFrontAngle(self):
        return self.StartingFrontAngle

    def getStartingRearAngle(self):
        return self.StartingRearAngle

    def getCorrectedFrontAngle(self):
        return [self.Angle[0] - self.StartingFrontAngle[0],
                self.Angle[1] - self.StartingFrontAngle[1],
                self.Angle[2] - self.StartingFrontAngle[2]]

    def getCorrectedRearAngle(self):
        return [self.Angle[0] - self.StartingRearAngle[0],
                self.Angle[1] - self.StartingRearAngle[1],
                self.Angle[2] - self.StartingRearAngle[2]]

    # end command/vehicle running
    def Terminate(self):
        self.serial.write("STOP")
        self.serial.close()


def parseXYZDataToList(xyz_data):
    i = -1
    xyz = [0.0, 0.0, 0.0]
    # xyz_data_clean = xyz_data
    # xyz_data_clean = xyz_data.replace('/n', '')
    for xyz_data_clean in str(xyz_data).split('\\'):
        for parsed in str(xyz_data_clean).split(':'):
            if 3 > i >= 0:
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
