#!python3
# Author: Theodor Giles
# Created: 7/15/20
# Last Edited 7/29/21
# Description:
# Node for data from the rpi

from Phidget22.Phidget import *
from Phidget22.Devices.Spatial import *
import time
import re
import math

GYRO: int = 0
POSITION: int = 1
YAW: int = 0
PITCH: int = 1
ROLL: int = 2
NORTH: int = 0
EAST: int = 1
DOWN: int = 2


def MapToAngle(x):
    in_min = -100.0
    in_max = 100.0
    out_min = 0.0
    out_max = 180.0
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class Phidget9dof:
    StringIn = ""
    Gyro = [0.0, 0.0, 0.0]
    Position = [0.0, 0.0, 0.0]
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

    def __init__(self):
        # read info from vehicle
        spatial0 = Spatial()

        spatial0.setOnSpatialDataHandler(self.onSpatialData)

        spatial0.openWaitForAttachment(5000)

        # arm vehicle to see position
        print('Gyro Armed')
        # - Read the actual attitude: Roll, Pitch, and Yaw
        self.UpdateGyro()
        self.StartingGyro = self.Gyro
        print('Orientation: ', self.getStartingGyro())

        # - Read the actual position North, East, and Down
        self.UpdatePosition()
        self.StartingPosition = self.Position
        print('Position: ', self.getStartingPosition())

        # - Read the actual depth:
        time.sleep(3)
        print("Starting gyro: ", self.StartingGyro)
        # print("Starting position: ", self.Position)

    def onSpatialData(self, acceleration, angularRate, magneticField, timestamp):
        print(
            "Acceleration: \t" + str(acceleration[0]) + "  |  " + str(acceleration[1]) + "  |  " + str(acceleration[2]))

        self.Position[NORTH] = acceleration[NORTH] * (timestamp**2)
        self.Position[EAST] =  acceleration[EAST] * (timestamp**2)
        self.Position[DOWN] = acceleration[DOWN] * (timestamp**2)

        print("AngularRate: \t" + str(angularRate[0]) + "  |  " + str(angularRate[1]) + "  |  " + str(angularRate[2]))
        print("MagneticField: \t" + str(magneticField[0]) + "  |  " + str(magneticField[1]) + "  |  " + str(
            magneticField[2]))
        print("Timestamp: " + str(timestamp))
        print("----------")

    # parse gyro object data from pixhawk, can then pass to other programs
    def UpdateGyro(self):
        pass

    # parse position object data from pixhawk, can then pass to other programs
    def UpdatePosition(self):
        pass

    # position read when starting the RoboSub
    def getStartingPosition(self):
        return self.StartingPosition

    # current position read
    def getPosition(self):
        return self.Position

    def getNorth(self):
        return self.Position[NORTH]

    def getEast(self):
        return self.Position[EAST]

    def getDown(self):
        return self.Position[DOWN]

    # gyro read when starting the RoboSub
    def getStartingGyro(self):
        return self.StartingGyro

    # current gyro read
    def getGyro(self):
        return self.Gyro

    def getPitch(self):
        return self.Gyro[PITCH]

    def getRoll(self):
        return self.Gyro[ROLL]

    def getYaw(self):
        return self.Gyro[YAW]

    # req for PID calculation
    def CalculateError(self, yawoffset, pitchoffset, rolloffset, northoffset, eastoffset, downoffset):
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
        self.Error[GYRO][YAW] = self.Gyro[YAW] - yawoffset
        self.Error[GYRO][PITCH] = self.Gyro[PITCH] - pitchoffset
        self.Error[GYRO][ROLL] = self.Gyro[ROLL] - rolloffset

        # position
        self.Error[POSITION][NORTH] = self.Position[NORTH] - northoffset
        self.Error[POSITION][EAST] = self.Position[EAST] - eastoffset
        self.Error[POSITION][DOWN] = self.Position[DOWN] - downoffset

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

    # end command/vehicle running
    def Terminate(self):
        self.serial.write("STOP")
        self.serial.close()
