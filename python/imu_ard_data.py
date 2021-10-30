#!python3
# Author: Theodor Giles
# Created: 7/15/20
# Last Edited 7/29/21
# Description:
# node for moving around IMU data from the arduino

import time
import re

GYRO: int = 0
POSITION: int = 1
YAW: int = 0
PITCH: int = 1
ROLL: int = 2
NORTH: int = 0
EAST: int = 1
DOWN: int = 2


class IMU:
    StringIn = ""
    Offsets = [0.0, 0.0, 0.0]
    StartingGyro = [0.0, 0.0, 0.0]
    Acceleration = [0.0, 0.0, 0.0]
    Velocity = [0.0, 0.0, 0.0]
    StartingPosition = [0.0, 0.0, 0.0]
    Angular_Motions = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Measures = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Previous_Error = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error_Sum = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error_Delta = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    # gyro              position

    # this is a comment
    Kp = [[0.7, 0.5, 0.5], [0.3, 0.4, 0.4]]  # constant to modify PID
    Ki = [[0.0, 0.00, 0.00], [0.1, 0.1, 0.1]]  # constant to modify PID
    Kd = [[0.3, 0.3, 0.3], [0.1, 0.1, 0.1]]  # constant to modify PID

    North_PID = 0.0
    North_P = 0.0
    North_I = 0.0
    North_D = 0.0

    East_PID = 0.0
    East_P = 0.0
    East_I = 0.0
    East_D = 0.0

    Down_PID = 0.0
    Down_P = 0.0
    Down_I = 0.0
    Down_D = 0.0

    Yaw_PID = 0.0
    Yaw_P = 0.0
    Yaw_I = 0.0
    Yaw_D = 0.0

    Pitch_PID = 0.0
    Pitch_P = 0.0
    Pitch_I = 0.0
    Pitch_D = 0.0

    Roll_PID = 0.0
    Roll_P = 0.0
    Roll_I = 0.0
    Roll_D = 0.0

    def __init__(self, arduino):

        # read info from vehicle
        self.serial = arduino
        self.serial.flushInput()

        # arm vehicle to see position
        # print(self.serial.readline())
        # - Read the actual attitude: Roll, Pitch, and Yaw
        self.UpdateGyro()
        self.StartingGyro = self.Offsets
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
        pass

    # parse gyro object data from wt61p, can then pass to other programs
    def UpdatePosition(self):
        pass

    # position read when starting the RoboSub
    def getStartingPosition(self):
        return self.StartingPosition

    # current position read
    def getPosition(self):
        return self.Acceleration

    def getNorth(self):
        return self.Acceleration[NORTH]

    def getEast(self):
        return self.Acceleration[EAST]

    def getDown(self):
        return self.Acceleration[DOWN]

    # gyro read when starting the RoboSub
    def getStartingGyro(self):
        return self.StartingGyro

    # current gyro read
    def getGyro(self):
        return self.Offsets

    def getPitch(self):
        return self.Offsets[PITCH]

    def getRoll(self):
        return self.Offsets[ROLL]

    def getYaw(self):
        return self.Offsets[YAW]

    # req for PID calculation
    def CalculateError(self, yawoffset, pitchoffset, rolloffset, northoffset, eastoffset, downoffset):

        self.Velocity[NORTH] = self.Acceleration[NORTH]
        # previous error for error delta
        # gyro
        self.Previous_Error[GYRO][YAW] = self.Error[GYRO][YAW]
        self.Previous_Error[GYRO][PITCH] = self.Error[GYRO][PITCH]
        self.Previous_Error[GYRO][ROLL] = self.Error[GYRO][ROLL]

        # position
        self.Previous_Error[POSITION][NORTH] = self.Error[POSITION][NORTH]
        self.Previous_Error[POSITION][EAST] = self.Error[POSITION][EAST]
        self.Previous_Error[POSITION][DOWN] = self.Error[POSITION][DOWN]

        # error for proportional control
        # gyro
        if ((180 - abs(yawoffset)) + (180 - abs(self.Offsets[YAW]))) < 180:
            self.Error[GYRO][YAW] = self.Offsets[YAW] - yawoffset
        elif ((abs(yawoffset)) + (abs(self.Offsets[YAW]))) < 180:
            self.Error[GYRO][YAW] = self.Offsets[YAW] + yawoffset
        self.Error[GYRO][PITCH] = self.Offsets[PITCH] - pitchoffset
        self.Error[GYRO][ROLL] = self.Offsets[ROLL] - rolloffset

        # position
        self.Error[POSITION][NORTH] = self.Acceleration[NORTH] - northoffset
        self.Error[POSITION][EAST] = self.Acceleration[EAST] - eastoffset
        self.Error[POSITION][DOWN] = self.Acceleration[DOWN] - downoffset

        # sum of error for integral
        # gyro
        self.Error_Sum[GYRO][YAW] = self.Error_Sum[GYRO][YAW] + self.Error[GYRO][YAW]
        self.Error_Sum[GYRO][PITCH] = self.Error_Sum[GYRO][PITCH] + self.Error[GYRO][PITCH]
        self.Error_Sum[GYRO][ROLL] = self.Error_Sum[GYRO][ROLL] + self.Error[GYRO][ROLL]

        # position
        self.Error_Sum[POSITION][NORTH] = self.Error_Sum[POSITION][NORTH] + self.Error[POSITION][NORTH]
        self.Error_Sum[POSITION][EAST] = self.Error_Sum[POSITION][EAST] + self.Error[POSITION][EAST]
        self.Error_Sum[POSITION][DOWN] = self.Error_Sum[POSITION][DOWN] + self.Error[POSITION][DOWN]

        # math for change in error to do derivative
        # gyro
        self.Error_Delta[GYRO][YAW] = self.Error[GYRO][YAW] - self.Previous_Error[GYRO][YAW]
        self.Error_Delta[GYRO][PITCH] = self.Error[GYRO][PITCH] - self.Previous_Error[GYRO][PITCH]
        self.Error_Delta[GYRO][ROLL] = self.Error[GYRO][ROLL] - self.Previous_Error[GYRO][ROLL]

        # position
        self.Error_Delta[POSITION][NORTH] = self.Error[POSITION][NORTH] - self.Previous_Error[POSITION][NORTH]
        self.Error_Delta[POSITION][EAST] = self.Error[POSITION][EAST] - self.Previous_Error[POSITION][EAST]
        self.Error_Delta[POSITION][DOWN] = self.Error[POSITION][DOWN] - self.Previous_Error[POSITION][DOWN]

    # pid calculation
    def PID(self):
        # Yaw PID variable setting
        self.Yaw_P = (self.Error[GYRO][YAW] * self.Kp[GYRO][YAW])
        self.Yaw_I = (self.Error_Sum[GYRO][YAW] * self.Ki[GYRO][YAW])
        self.Yaw_D = (self.Error_Delta[GYRO][YAW] * self.Kd[GYRO][YAW])
        self.Yaw_PID = self.Yaw_P + self.Yaw_I + self.Yaw_D

        # Pitch PID variable setting
        self.Pitch_P = (self.Error[GYRO][PITCH] * self.Kp[GYRO][PITCH])
        self.Pitch_I = (self.Error_Sum[GYRO][PITCH] * self.Ki[GYRO][PITCH])
        self.Pitch_D = (self.Error_Delta[GYRO][PITCH] * self.Kd[GYRO][PITCH])
        self.Pitch_PID = self.Pitch_P + self.Pitch_I + self.Pitch_D

        # Roll PID variable setting
        self.Roll_P = (self.Error[GYRO][ROLL] * self.Kp[GYRO][ROLL])
        self.Roll_I = (self.Error_Sum[GYRO][ROLL] * self.Ki[GYRO][ROLL])
        self.Roll_D = (self.Error_Delta[GYRO][ROLL] * self.Kd[GYRO][ROLL])
        self.Roll_PID = self.Roll_P + self.Roll_I + self.Roll_D

        # North PID variable setting
        self.North_P = (self.Error[POSITION][NORTH] * self.Kp[POSITION][NORTH])
        self.North_I = (self.Error_Sum[POSITION][NORTH] * self.Ki[POSITION][NORTH])
        self.North_D = (self.Error_Delta[POSITION][NORTH] * self.Kd[POSITION][NORTH])
        self.North_PID = self.North_P  # + self.North_I + self.North_D

        # East PID variable setting
        self.East_P = (self.Error[POSITION][EAST] * self.Kp[POSITION][EAST])
        self.East_I = (self.Error_Sum[POSITION][EAST] * self.Ki[POSITION][EAST])
        self.East_D = (self.Error_Delta[POSITION][EAST] * self.Kd[POSITION][EAST])
        self.East_PID = self.East_P  # + self.East_I + self.East_D

        # Down PID variable setting
        self.Down_P = (self.Error[POSITION][DOWN] * self.Kp[POSITION][DOWN])
        self.Down_I = (self.Error_Sum[POSITION][DOWN] * self.Ki[POSITION][DOWN])
        self.Down_D = (self.Error_Delta[POSITION][DOWN] * self.Kd[POSITION][DOWN])
        self.Down_PID = self.Down_P  # + self.Down_I + self.Down_D

    def getYawPID(self):
        return self.Yaw_PID

    def getPitchPID(self):
        return self.Pitch_PID

    def getRollPID(self):
        return self.Roll_PID

    def getNorthPID(self):
        return self.Yaw_PID

    def getEastPID(self):
        return self.Pitch_PID

    def getDownPID(self):
        return self.Roll_PID


class WT61P(IMU):

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
                    self.Offsets[ROLL] = float(ColonParse[0])
                if i == 4:
                    self.Offsets[PITCH] = float(ColonParse[0])
                if i == 6:
                    self.Offsets[YAW] = float(ColonParse[0])
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
                    self.Offsets[DOWN] = float(ColonParse[0])
                if i == 4:
                    self.Offsets[EAST] = float(ColonParse[0])
                if i == 6:
                    self.Offsets[NORTH] = float(ColonParse[0])
                    break
            i = i + 1
        pass

    # end command/vehicle running
    def Terminate(self):
        self.serial.write("STOP")
        self.serial.close()


class BN055(IMU):

    def __init__(self, arduino):

        # read info from vehicle
        self.serial = arduino
        self.serial.flushInput()

        # arm vehicle to see position
        # print(self.serial.readline())
        # - Read the actual attitude: Roll, Pitch, and Yaw
        self.UpdateGyro()
        self.StartingGyro = self.Offsets
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
        for ColonParse in line:
            if ColonParse is not None:
                ColonParse = re.findall(r"[-+]?\d*\.\d+|\d+", ColonParse)
                # print("ColonParse: ", ColonParse, i)
                if i == 2:
                    self.Offsets[YAW] = float(ColonParse[0])
                if i == 4:
                    self.Offsets[PITCH] = float(ColonParse[0])
                if i == 6:
                    self.Offsets[ROLL] = float(ColonParse[0])
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
