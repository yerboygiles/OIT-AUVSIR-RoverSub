#!python3
# Author: Theodor Giles
# Created: 7/15/20
# Last Edited 7/29/21
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



class WT61P(IMU):

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
        self.serial.writelines("GYRO")
        line = str(self.serial.readline()).strip("'").split(':')
        i = 0
        for ColonParse in line:
            if ColonParse is not None:
                ColonParse = re.findall(r"[-+]?\d*\.\d+|\d+", ColonParse)
                # print("ColonParse: ", ColonParse, i)
                if i == 2:
                    self.Angle[ROLL] = float(ColonParse[0])
                if i == 4:
                    self.Angle[PITCH] = float(ColonParse[0])
                if i == 6:
                    self.Angle[YAW] = float(ColonParse[0])
                    break
            i = i + 1
        pass

    # parse position object data from wt61p, can then pass to other programs
    def UpdatePosition(self):
        self.serial.writelines("POSITION")
        line = str(self.serial.readline()).strip("'").split(':')
        i = 0
        for ColonParse in line:
            if ColonParse is not None:
                ColonParse = re.findall(r"[-+]?\d*\.\d+|\d+", ColonParse)
                # print("ColonParse: ", ColonParse, i)
                if i == 2:
                    self.Angle[DOWN] = float(ColonParse[0])
                if i == 4:
                    self.Angle[EAST] = float(ColonParse[0])
                if i == 6:
                    self.Angle[NORTH] = float(ColonParse[0])
                    break
            i = i + 1
        pass

    # end command/vehicle running
    def Terminate(self):
        self.serial.write("STOP")
        self.serial.close()


class BN055(IMU):

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

    # parse gyro object data from pixhawk, can then pass to other programs
    def UpdateGyro(self):
        self.serial.writelines("GYRO")
        i = 0
        time.sleep(0.01)
        # print("Updating...")
        line = str(self.serial.readline()).strip("'").split(':')
        for colonparse in line:
            if colonparse is not None:
                colonparse = re.findall(r"[-+]?\d*\.\d+|\d+", colonparse)
                # print("ColonParse: ", ColonParse, i)
                if i == 2:
                    self.Angle[YAW] = float(colonparse[0])
                if i == 4:
                    self.Angle[PITCH] = float(colonparse[0])
                if i == 6:
                    self.Angle[ROLL] = float(colonparse[0])
                    break
            i = i + 1
        # print("Gyro: ", self.Gyro)

    # parse position object data from pixhawk, can then pass to other programs
    def UpdatePosition(self):
        self.serial.writelines("POSITION")
        i = 0
        time.sleep(0.01)
        line = str(self.serial.readline()).strip("'").split(':')
        for ColonParse in line:
            if ColonParse is not None:
                ColonParse = re.findall(r"[-+]?\d*\.\d+|\d+", ColonParse)
                if ColonParse == "Orient":
                    break
                if i == 2:
                    self.Acceleration[NORTH] = float(ColonParse[0])
                if i == 4:
                    self.Acceleration[EAST] = float(ColonParse[0])
                if i == 6:
                    self.Acceleration[DOWN] = float(ColonParse[0])
                    break
            i = i + 1

    def WriteToSerial(self, toprint):
        self.serial.writelines(toprint)

    # end command/vehicle running
    def Terminate(self):
        self.serial.write("STOP")
        self.serial.close()


class ArduinoHandler(IMU):

    def __init__(self, serial, numWT61P=0, numBN055=0):
        self.WT61P_list = []
        self.BN055_list = []
        for i in range(numWT61P):
            self.WT61P_list[i] = WT61P(self.serial, i)
        for i in range(numBN055):
            self.BN055_list[i] = BN055(self.serial, i)
