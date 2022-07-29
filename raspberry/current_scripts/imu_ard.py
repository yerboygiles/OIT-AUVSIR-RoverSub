#!python3
# Author: Theodor Giles, Colton Rhodes
# Created: 7/15/20
# Last Edited 7/23/22
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
    StartingLeftAngle = [0.0, 0.0, 0.0]
    StartingCenterAngle = [0.0, 0.0, 0.0]
    StartingRightAngle = [0.0, 0.0, 0.0]
    FrontAngle = [0.0, 0.0, 0.0]
    RearAngle = [0.0, 0.0, 0.0]
    Position = [0.0, 0.0, 0.0]
    Kp = [[0.4, 0.4, 0.4], [0.3, 0.4, 0.4]]  # constant to modify PID
    Ki = [[0.05, 0.15, 0.15], [0.1, 0.1, 0.1]]  # constant to modify PID
    Kd = [[0.2, 0.2, 0.2], [0.1, 0.1, 0.1]]  # constant to modify PID

    integral_overflow = 0

    def __init__(self, serial):
        # read info from vehicle
        self.serial = serial
        self.serial.flushInput()

        # arm vehicle to see position
        # print(self.serial.readline())
        # - Read the actual attitude: Roll, Pitch, and Yaw
        # time.sleep(3)
        self.CalibrateStart()
        print("Starting Center Angle: ", self.StartingCenterAngle)
        print("Starting Right Angle: ", self.StartingRightAngle)
        print("Starting Left Angle: ", self.StartingLeftAngle)
        # print('Orientation: ', self.getStartingAngle())

        # - Read the actual position North, East, and Down
        # self.UpdatePosition()
        # self.StartingPosition = self.Position
        # print('Position: ', self.getStartingPosition())

        # - Read the actual depth:
        time.sleep(3)
        self.dt = 0
        self.storedAcceleration = [0.0, 0.0, 0.0]
        # print("Starting position: ", self.Position)

    # parse gyro object data from wt61p, can then pass to other programs
    # def UpdateAngle(self):
    #
    #     anglefront = self.parseAngleFront()
    #     anglerear = self.parseAngleRear()
    #     startfront = self.getStartingFrontAngle()
    #     startrear = self.getStartingRearAngle()
    #
    #     # anglerear[0]
    #
    #     self.Angle[0] = round(((anglefront[0] - startfront[0]) + (anglerear[0] - startrear[0])) / 2, 4)
    #     self.Angle[1] = round(((anglefront[1] - startfront[1]) - (anglerear[1] - startrear[1])) / 2, 4)
    #     self.Angle[2] = round(((anglefront[2] - startfront[2]) - (anglerear[2] - startrear[2])) / 2, 4)

    def UpdateIMU(self):
        self.UpdateAngle()
        self.UpdatePosition()

    def UpdateAngle(self):

        startcenter = self.getStartingCenterAngle()
        startright = self.getStartingRightAngle()
        # startleft = self.getStartingLeftAngle()
        anglecenter, anglerear = self.parseAngles()

        self.Angle[YAW] = ((round(((anglecenter[0] - startcenter[0]) + (anglerear[0] - startright[0])) / 2, 4) % 360) +
                           360) % 360
        self.Angle[PITCH] = round(((anglecenter[1] - startcenter[1]) + (anglerear[1] - startright[1])) / 2, 4)
        self.Angle[ROLL] = round(((anglecenter[2] - startcenter[2]) + (anglerear[2] - startright[2])) / 2, 4)
        print("Angles: ", self.Angle)

    # parse position object data from wt61p, can then pass to other programs
    def UpdatePosition(self, verlet=True):
        self.dt = time.perf_counter() - self.dt
        if self.dt > 2000:
            self.dt = 6

        # print("Accel: ", self.Acceleration)
        i = 0
        for velocity in self.Velocity:
            self.Velocity[i] = round((self.storedAcceleration[i] + self.Acceleration[i]) * self.dt, 4)
            i = i + 1
        i = 0
        # print("Velocity: ", self.Velocity)
        if verlet:
            # verlet integration
            for position in self.Position:
                self.Position[i] = round(
                    position + self.Velocity[i] * self.dt + 0.5 * (self.Acceleration[i] * self.dt ** 2) + position, 4)
                i = i + 1
        else:
            # euler integration
            for position in self.Position:
                self.Position[i] = self.Velocity[i] * self.dt - position
                i = i + 1
        self.storedAcceleration = self.Acceleration
        pass
        # print("Position: ", self.Position)
        self.dt = time.perf_counter() - self.dt

    def UpdateAcceleration(self):
        accelfront, accelrear = self.parseAccelFrontAndRear()

        self.Acceleration[0] = round((((accelfront[0]) + (accelrear[0])) / 2), 4)
        self.Acceleration[1] = round((((accelfront[1]) + (accelrear[1])) / 2), 4)
        self.Acceleration[2] = round((((accelfront[2]) + (accelrear[2])) / 2) - 0.99, 4)

    def CalibrateStart(self):
        angle = self.parseAngleFront()
        i = 0
        for x in angle:
            angle[i] = round(x, 4)
            i = i + 1
        self.StartingCenterAngle = angle
        angle = self.parseAngleRear()
        i = 0
        for x in angle:
            angle[i] = round(x, 4)
            i = i + 1
        self.StartingRightAngle = angle

        angle = self.parseAngleFront()
        i = 0
        for x in angle:
            angle[i] = round(x, 4)
            i = i + 1
        self.StartingCenterAngle = angle
        angle = self.parseAngleRear()
        i = 0
        for x in angle:
            angle[i] = round(x, 4)
            i = i + 1
        self.StartingRightAngle = angle

    def UpdateFrontAngle(self):
        angleFront = self.parseAngleFront()
        angle = [0.0, 0.0, 0.0]
        self.Angle[0] = (angleFront[0])
        self.Angle[1] = (angleFront[1])
        self.Angle[2] = (angleFront[2])
        return self.Angle
        # print("Front Angle: ", self.Angle)

    def UpdateRearAngle(self):
        anglerear = self.parseAngleRear()
        self.Angle[0] = (anglerear[0])
        self.Angle[1] = (anglerear[1])
        self.Angle[2] = (anglerear[2])
        return self.Angle
        # print("Rear Angle: ", self.Angle)

    def parseAccelFrontAndRear(self):
        self.serial.write("gca\n".encode('ascii'))
        data = self.serial.read_until("\n")
        # time.sleep(0.05)
        accelerations = self.parseDoubleXYZDataToList(data)
        return accelerations

    def parseAngles(self):
        self.serial.write("gaa\n".encode('ascii'))
        data = self.serial.read_until("\n")
        # time.sleep(0.05)
        angles = self.parseDoubleXYZDataToList(data)
        return angles

    def parseAngleFront(self):
        self.serial.write("gfa\n".encode('ascii'))
        data = self.serial.read_until("\n")
        # time.sleep(0.05)
        angles = self.parseXYZDataToList(data)
        return angles

    def parseAngleRear(self):
        self.serial.write("gra\n".encode('ascii'))
        data = self.serial.read_until("\n")
        # time.sleep(0.05)
        angles = self.parseXYZDataToList(data)
        # if angles[0] < 0:
        #     angles[0] = angles[0] - 180
        # if angles[0] > 0:
        #     angles[0] = angles[0] + 180
        return angles

    def getStartingCenterAngle(self):
        return self.StartingCenterAngle

    def getStartingRightAngle(self):
        return self.StartingRightAngle

    def getCorrectedFrontAngle(self):
        return [self.Angle[0] - self.StartingCenterAngle[0],
                self.Angle[1] - self.StartingCenterAngle[1],
                self.Angle[2] - self.StartingCenterAngle[2]]

    def getCorrectedRearAngle(self):
        return [self.Angle[0] - self.StartingRightAngle[0],
                self.Angle[1] - self.StartingRightAngle[1],
                self.Angle[2] - self.StartingRightAngle[2]]

    def getCorrectedLeftAngle(self):
        return [self.Angle[0] - self.StartingLeftAngle[0],
                self.Angle[1] - self.StartingLeftAngle[1],
                self.Angle[2] - self.StartingLeftAngle[2]]

    # req for PID calculation
    def CalculateError(self, yawoffset=0, pitchoffset=0, rolloffset=0, northoffset=0, eastoffset=0, downoffset=0):

        # previous error for error delta
        # gyro
        self.Previous_Error[GYRO][YAW] = self.Error[GYRO][YAW]
        self.Previous_Error[GYRO][PITCH] = self.Error[GYRO][PITCH]
        self.Previous_Error[GYRO][ROLL] = self.Error[GYRO][ROLL]

        # position
        # self.Previous_Error[POSITION][NORTH] = self.Error[POSITION][NORTH]
        # self.Previous_Error[POSITION][EAST] = self.Error[POSITION][EAST]
        # self.Previous_Error[POSITION][DOWN] = self.Error[POSITION][DOWN]

        # error for proportional control
        # gyro
        # brand new calcs
        self.Error[GYRO][YAW] = self.Angle[YAW] - yawoffset
        self.Error[GYRO][PITCH] = self.Angle[PITCH] - pitchoffset
        self.Error[GYRO][ROLL] = self.Angle[ROLL] - rolloffset
        # right side, going left side
        if abs(self.Error[GYRO][YAW]) < 180:
            if self.Angle[YAW] < yawoffset:
                self.Error[GYRO][YAW] = self.Error[GYRO][YAW]
            else:
                self.Error[GYRO][YAW] = -self.Error[GYRO][YAW]
        # left side, going right
        else:
            if self.Angle[YAW] < yawoffset:
                self.Error[GYRO][YAW] = -self.Error[GYRO][YAW]
            else:
                self.Error[GYRO][YAW] = self.Error[GYRO][YAW]
        # print("Yaw: ", self.Angle[YAW])

        # position
        # self.Error[POSITION][NORTH] = self.Acceleration[NORTH] - northoffset
        # self.Error[POSITION][EAST] = self.Acceleration[EAST] - eastoffset
        # self.Error[POSITION][DOWN] = self.Acceleration[DOWN] - downoffset

        # sum of error for integral
        # gyro
        if abs(self.Error_Sum[GYRO][YAW]) > abs(self.Error[GYRO][YAW] * 50) or abs(self.Error[GYRO][YAW]) < 3:
            self.Error_Sum[GYRO][YAW] = 0
        else:
            self.Error_Sum[GYRO][YAW] = self.Error_Sum[GYRO][YAW] + self.Error[GYRO][YAW]
        self.Error_Sum[GYRO][PITCH] = self.Error_Sum[GYRO][PITCH] + self.Error[GYRO][PITCH]
        self.Error_Sum[GYRO][ROLL] = self.Error_Sum[GYRO][ROLL] + self.Error[GYRO][ROLL]

        # position
        # self.Error_Sum[POSITION][NORTH] = self.Error_Sum[POSITION][NORTH] + self.Error[POSITION][NORTH]
        # self.Error_Sum[POSITION][EAST] = self.Error_Sum[POSITION][EAST] + self.Error[POSITION][EAST]
        # self.Error_Sum[POSITION][DOWN] = self.Error_Sum[POSITION][DOWN] + self.Error[POSITION][DOWN]

        # math for change in error to do derivative
        # gyro
        self.Error_Delta[GYRO][YAW] = self.Error[GYRO][YAW] - self.Previous_Error[GYRO][YAW]
        self.Error_Delta[GYRO][PITCH] = self.Error[GYRO][PITCH] - self.Previous_Error[GYRO][PITCH]
        self.Error_Delta[GYRO][ROLL] = self.Error[GYRO][ROLL] - self.Previous_Error[GYRO][ROLL]

        # position
        # self.Error_Delta[POSITION][NORTH] = self.Error[POSITION][NORTH] - self.Previous_Error[POSITION][NORTH]
        # self.Error_Delta[POSITION][EAST] = self.Error[POSITION][EAST] - self.Previous_Error[POSITION][EAST]
        # self.Error_Delta[POSITION][DOWN] = self.Error[POSITION][DOWN] - self.Previous_Error[POSITION][DOWN]

    # pid calculation
    # end command/vehicle running
    def Terminate(self):
        self.serial.write("STOP")
        self.serial.close()

    def parseXYZDataToList(self, xyz_data):
        i = -1
        xyz = [0.0, 0.0, 0.0]
        # xyz_data_clean = xyz_data
        # xyz_data_clean = xyz_data.replace('/n', '')
        # print(xyz_data)
        try:
            for xyz_data_clean in str(xyz_data).split('\\'):
                for parsed in str(xyz_data_clean).split(':'):
                    if 3 > i >= 0:
                        xyz[i] = float(parsed)
                    i = i + 1
        except:
            print("Error parsing angle data.")
            xyz = self.Angle
        return xyz

    def parseDoubleXYZDataToList(self, xyz_data):
        i = -1
        xyz = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        # xyz_data_clean = xyz_data
        # xyz_data_clean = xyz_data.replace('/n', '')
        # print(xyz_data)
        try:
            for xyz_data_clean in str(xyz_data).split('\\'):
                for semiparsed in str(xyz_data_clean).split(':'):
                    if 3 > i >= 0:
                        j = 0
                        for fullparsed in str(semiparsed).split(','):
                            xyz[j][i] = float(fullparsed)
                            j = j + 1
                    i = i + 1
        except:
            print("Error parsing angle data.")
            xyz = self.Angle
        return xyz

    def PID(self):
        if self.integral_overflow > 30:
            self.Error_Sum[GYRO][YAW] = 0
            self.Error_Sum[GYRO][PITCH] = 0
            self.Error_Sum[GYRO][ROLL] = 0
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

        self.integral_overflow = self.integral_overflow + 1
        # # North PID variable setting
        # self.North_P = (self.Error[POSITION][NORTH] * self.Kp[POSITION][NORTH])
        # self.North_I = (self.Error_Sum[POSITION][NORTH] * self.Ki[POSITION][NORTH])
        # self.North_D = (self.Error_Delta[POSITION][NORTH] * self.Kd[POSITION][NORTH])
        # self.North_PID = self.North_P  # + self.North_I + self.North_D
        #
        # # East PID variable setting
        # self.East_P = (self.Error[POSITION][EAST] * self.Kp[POSITION][EAST])
        # self.East_I = (self.Error_Sum[POSITION][EAST] * self.Ki[POSITION][EAST])
        # self.East_D = (self.Error_Delta[POSITION][EAST] * self.Kd[POSITION][EAST])
        # self.East_PID = self.East_P  # + self.East_I + self.East_D
        #
        # # Down PID variable setting
        # self.Down_P = (self.Error[POSITION][DOWN] * self.Kp[POSITION][DOWN])
        # self.Down_I = (self.Error_Sum[POSITION][DOWN] * self.Ki[POSITION][DOWN])
        # self.Down_D = (self.Error_Delta[POSITION][DOWN] * self.Kd[POSITION][DOWN])
        # self.Down_PID = self.Down_P  # + self.Down_I + self.Down_D

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
