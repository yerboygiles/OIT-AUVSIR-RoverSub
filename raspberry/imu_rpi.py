#!python3
# Author: Theodor Giles
# Created: 7/15/20
# Last Edited 7/29/21
# Description:
# Node for data from the rpi

import time
from Phidget22.Phidget import *
from Phidget22.Devices.Spatial import *
import numpy as np
from numpy import pi
import serial

from raspberry.current_scripts.imu import IMU

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


class Phidget9dof(IMU):

    def __init__(self):
        # read info from vehicle
        spatial0 = Spatial()

        spatial0.setOnSpatialDataHandler(self.onSpatialData)

        spatial0.openWaitForAttachment(5000)

        # arm vehicle to see position
        print('Gyro Armed')
        # - Read the actual attitude: Roll, Pitch, and Yaw
        self.UpdateGyro()
        self.StartingGyro = self.Angle
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

        self.Position[NORTH] = acceleration[NORTH] * (timestamp ** 2)
        self.Position[EAST] = acceleration[EAST] * (timestamp ** 2)
        self.Position[DOWN] = acceleration[DOWN] * (timestamp ** 2)

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
        self.Error[GYRO][YAW] = self.Angle[YAW] - yawoffset
        self.Error[GYRO][PITCH] = self.Angle[PITCH] - pitchoffset
        self.Error[GYRO][ROLL] = self.Angle[ROLL] - rolloffset

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


class JY62(IMU):
    # acceleration
    Acceleration = [0.0] * 3
    # angular velocity
    AngularVelocity = [0.0] * 3
    # angle
    Angle = [0.0, 0.0, 0.0]
    # angle_in_quat
    Angle_quat = [0.0] * 4

    ACCData = [0.0] * 8
    GYROData = [0.0] * 8
    AngleData = [0.0] * 8

    FrameState = 0
    Bytenum = 0
    CheckSum = 0

    def __init__(self, port, id=0):

        # read info from vehicle
        self.port = port
        self.serial = serial.Serial(self.port, 9600)
        self.serial.flushInput()
        self.ID = id

        # arm vehicle to see position
        # print(self.serial.readline())
        # - Read the actual attitude: Roll, Pitch, and Yaw
        print("Initializing Gyro, ID - ", self.ID)
        self.updateGyro()
        self.StartingAngle = self.Angle
        print('Orientation: ', self.getStartingGyro())

        # - Read the actual position North, East, and Down
        # self.UpdatePosition()
        # self.StartingPosition = self.Position
        # print('Position: ', self.getStartingPosition())

        # - Read the actual depth:
        time.sleep(3)
        print("Starting gyro: ", self.StartingAngle)
        # print("Starting position: ", self.Position)

    def resetGyro(self):
        # '/dev/ttyUSB0'
        self.serial.close()
        # save this for when we dynamically connect/disconnect gyros
        # connectstring = '/dev/ttyUSB' + str(self.ID)
        # print(connectstring)
        self.serial = serial.Serial(self.port, 115200)

    def calibrateGyro(self):
        pass

    def updateGyro(self):
        datahex = self.serial.read(33)
        self.DueData(datahex)

    # reading info in correct format from jy62
    def DueData(self, inputdata):

        for data in inputdata:
            # data = ord(data)
            if self.FrameState == 0:
                if data == 0x55 and self.Bytenum == 0:
                    self.CheckSum = data
                    self.Bytenum = 1
                    continue
                elif data == 0x51 and self.Bytenum == 1:
                    self.CheckSum += data
                    self.FrameState = 1
                    self.Bytenum = 2
                elif data == 0x52 and self.Bytenum == 1:
                    self.CheckSum += data
                    self.FrameState = 2
                    self.Bytenum = 2
                elif data == 0x53 and self.Bytenum == 1:
                    self.CheckSum += data
                    self.FrameState = 3
                    self.Bytenum = 2
            elif self.FrameState == 1:  # acc

                if self.Bytenum < 10:
                    self.ACCData[self.Bytenum - 2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.Acceleration = get_acc(self.ACCData)
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 2:  # gyro

                if self.Bytenum < 10:
                    self.GYROData[self.Bytenum - 2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.AngularVelocity = get_gyro(self.GYROData)
                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0
            elif self.FrameState == 3:  # angle

                if self.Bytenum < 10:
                    self.AngleData[self.Bytenum - 2] = data
                    self.CheckSum += data
                    self.Bytenum += 1
                else:
                    if data == (self.CheckSum & 0xff):
                        self.Angle = get_angle(self.AngleData)
                        self.Angle_quat = euler_to_quaternion(self.Angle[0], self.Angle[1], self.Angle[2])

                        # change acceleration to m/s2 (by default the unit is g)
                        self.Acceleration = [9.81 * i for i in self.Acceleration]
                        self.Acceleration = tuple(self.Acceleration)

                        # print
                        # "acceleration(m/s2):\t%10.3f %10.3f %10.3f" % self.Acceleration
                        # print
                        # "angular vel(deg/s):\t%10.3f %10.3f %10.3f" % self.AngularVelocity
                        # print
                        # "angle(deg):\t\t%10.3f %10.3f %10.3f" % self.Angle
                        # print
                        # "angle_quat:\t\t%10.3f %10.3f %10.3f %10.3f" % self.Angle_quat
                        # print
                        # "====================================================================="

                    self.CheckSum = 0
                    self.Bytenum = 0
                    self.FrameState = 0

    def Terminate(self):
        self.serial.close()

# fns for JY61
def get_gyro(datahex):
    wxl = datahex[0]
    wxh = datahex[1]
    wyl = datahex[2]
    wyh = datahex[3]
    wzl = datahex[4]
    wzh = datahex[5]
    k_gyro = 2000.0

    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >= k_gyro:
        gyro_z -= 2 * k_gyro
    return gyro_x, gyro_y, gyro_z


def get_acc(datahex):
    axl = datahex[0]
    axh = datahex[1]
    ayl = datahex[2]
    ayh = datahex[3]
    azl = datahex[4]
    azh = datahex[5]

    k_acc = 16.0

    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z -= 2 * k_acc

    return acc_x, acc_y, acc_z


def get_angle(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    k_angle = 180.0

    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >= k_angle:
        angle_z -= 2 * k_angle

    return angle_x, angle_y, angle_z


def euler_to_quaternion(roll, pitch, yaw):
    # change to radius
    yaw = yaw * pi / 180
    pitch = pitch * pi / 180
    roll = roll * pi / 180

    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    return qx, qy, qz, qw

# def searchUSBSerial():
#     for ns in xrange(101):
#         try:
#             ser.port = ns
#             ser.open()
#             print
#             "COM" + str(ns + 1) + " available"
#             ser.close()
#
#         except serial.SerialException:
#             print
#             "COM" + str(ns + 1) + " NOT available"
