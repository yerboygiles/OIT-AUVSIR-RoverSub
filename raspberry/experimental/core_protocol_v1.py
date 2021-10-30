#!python3
# Author: Theodor Giles
# Created: 11/22/20
# Last Edited 5/26/21
# Description:
# This program manages the commands/movement/physical
# control of the RoboSub V2, 2020-21

import time
# import random
import math
import serial
import bno055_data
import phidget9dof_data
import gyro_data_merger
import remote_control

# from threading import Thread

# ROBOSUB
A_TARGET = 1
A_POSITION = 2
A_GYRO = 3

MAX_THROTTLE = 15

GENERAL_THROTTLE = 17.5


class MovementCommander:

    # initialize everything to supposed starting position
    def __init__(self, usingvision=False, usinggyro=False, usingsim=False, resetheadingoncmd=False):
        # setting up board serial port
        print("Waiting 8 for Arduino...")
        time.sleep(8)
        print("Communicating with Arduino and it's peripherals...")
        self.UsingGyro = usinggyro
        self.serial = serial.Serial('/dev/ttyAMA0', 115200)
        if self.UsingGyro:
            print("Sending IMU")
            self.SendToArduino("IMU")
            self.Gyro_drone1 = bno055_data.BN055(self.serial)
            self.Gyro_queen = phidget9dof_data.Phidget9dof()
            self.Gyro_hive = gyro_data_merger.GyroMerger(self.Gyro_queen, self.Gyro_drone1)
        else:
            print("Sending NOIMU")
            self.SendToArduino("NOIMU")

        if resetheadingoncmd:
            self.YawOffset = self.Gyro_hive.StartingGyro[0]
            self.ResetHeadingOnCMD = resetheadingoncmd
        self.YawOffset = 0
        self.PitchOffset = 0
        self.RollOffset = 0

        self.EastOffset = 0
        self.NorthOffset = 0
        self.DownOffset = 0

        self.UsingVision = usingvision
        self.UsingSim = usingsim
        if self.UsingVision:
            import Theos_Really_Good_Detection_Script as obj_det
            self.VisionAI = obj_det.Detector("TensorFlow_Graph/Tflite", False)
            print("MovementCommander is using Vision AI...")
        else:
            print("MovementCommander is not using Vision AI...")

        if self.UsingSim:
            from python.experimental.advanced_telemetry import Telemetry
            self.TelemetrySim = Telemetry()
            print("MovementCommander is using Telemetry...")
        else:
            print("MovementCommander is not using Telemetry...")
        # thruster hardpoint classes
        # 'ventral' are the central, vertically oriented thrusters
        # for roll/pitch and ascent/descent
        # 'lateral' are the outer, 45 deg. oriented thrusters for
        # yaw/turning and strafe movement
        self.VentralThrusterLB = ThrusterDriver("LB")  # left back
        self.VentralThrusterLF = ThrusterDriver("LF")  # left front
        self.VentralThrusterRB = ThrusterDriver("RB")  # right back
        self.VentralThrusterRF = ThrusterDriver("RF")  # right front
        self.LateralThrusterLB = ThrusterDriver("BL")  # back left
        self.LateralThrusterRB = ThrusterDriver("BR")  # back right
        self.LateralThrusterLF = ThrusterDriver("FL")  # front left !
        self.LateralThrusterRF = ThrusterDriver("FR")  # front right !
        # power values to set to the thruster hardpoints
        # horizontally oriented
        self.LateralPowerLB = 0
        self.LateralPowerLF = 0
        self.LateralPowerRB = 0
        self.LateralPowerRF = 0
        # vertically oriented
        self.VentralPowerLB = 0
        self.VentralPowerRB = 0
        self.VentralPowerRF = 0
        self.VentralPowerLF = 0
        # boolean for toggling between data sent
        self.secondSetTrade = False
        # initialize thruster values to brake (self.PowerXX set to 0^)
        self.UpdateThrusters()

        # string list of movement commands, because I thought I'd make
        # the index number of each command streamlined with other
        # functions, but it seems a bit detrimental the more I work with
        # it.
        # advanced: these commands are much more complicated, will need to
        # develop pathing and a lot of vision/gyro/position integration
        self.BASIC_MOVEMENT_COMMANDS = [
            "FORWARDS",
            "BACKWARDS",
            "LEFT",
            "RIGHT"
        ]
        self.ADVANCED_MOVEMENT_COMMANDS = [
            "LOG START POINT",
            "RETURN TO START",
            "GYRO TO",
            "POSITION TO",
            "SCAN FOR TARGET"
        ]
        self.TARGET_MOVEMENT_COMMANDS = [
            "MOVE",
            "RAM",
            "FIRE AT",
            "FOLLOW"
        ]
        # currently only for firing torpedoes, maybe a claw action later on?
        self.ACTION_COMMANDS = [
            "FIRE TORPEDO AT",
            "GRAB OBJECT",
            ""
        ]
        # name of object to target sent to TF/openCV AI
        self.TO_TARGET = ""

        # possible targets, matches up with labelmap.txt to make easier
        self.POSSIBLE_TARGETS = [
            "red_buoy",
            "blue_buoy",
            "green_buoy",
            "orange_buoy",
            "gate",
            "Cesar"]
        self.TargetList = []
        print("MovementCommander initialized...")

    def BasicDriverControl(self):
        DrivingWithControl = True
        print("Driver Control!!")
        while DrivingWithControl:
            DriveCommand = remote_control.get_wasdqerv_directional()
            self.BasicDirectionPower(DriveCommand)
            DrivingWithControl = DriveCommand is not -2
            self.TradeWithArduino()

    def BasicLinear(self, supplemental):
        pass

    def BasicVectoring(self, supplemental):
        Vectoring = True
        i = 0
        print("Supplemental: ", supplemental)
        for SuppParse in str(supplemental).split(':'):
            print("SuppParse: ", SuppParse)
            if i == 0:
                self.YawOffset = float(SuppParse)
                # print("YawOffset: ", self.YawOffset)
            if i == 1:
                self.PitchOffset = float(SuppParse)
            if i == 2:
                self.RollOffset = float(SuppParse)
            if i > 2:
                break
            i = i + 1
        while Vectoring:
            self.CheckIfGyroDone(threshold=10, timethreshold=3)
            Vectoring = self.GyroRunning
            self.TradeWithArduino()

    def AdvancedVectoring(self):
        pass

    # Concept memory
    def IsTargetInMemory(self, label, x, y, z):
        NewTarget = [label, x, y, z]
        InMemory = False
        for target in self.TargetList:
            # Determining how far something could be next to the said target,
            DistanceConfidence = math.sqrt(target[4]) * 1.5
            WithinX = abs(NewTarget[1] - target[1]) > DistanceConfidence
            WithinY = abs(NewTarget[2] - target[2]) > DistanceConfidence
            WithinZ = abs(NewTarget[3] - target[3]) > DistanceConfidence
            if (target[0] != NewTarget[0]) and WithinX and WithinY and WithinZ:
                InMemory = True
        return InMemory

    # Concept code, puts target into memory
    def SaveTargetToMemory(self, label, x, y, z, area):
        TargetInfo = [label, x, y, z, area]
        self.TargetList.append(TargetInfo)

    def SendToArduino(self, message):
        self.serial.write(message.encode('utf-8'))

    def TradeWithArduino(self):
        self.UpdateThrusters()
        outdata = ""
        if self.secondSetTrade:
            outdata += str(self.LateralThrusterLB.name)
            outdata += ":"
            outdata += str(self.LateralThrusterLB.GetSpeed())
            outdata += ","
            outdata += str(self.LateralThrusterRB.name)
            outdata += ":"
            outdata += str(self.LateralThrusterRB.GetSpeed())
            outdata += ","
            outdata += str(self.LateralThrusterLF.name)
            outdata += ":"
            outdata += str(self.LateralThrusterLF.GetSpeed())
            outdata += ","
            outdata += str(self.LateralThrusterRF.name)
            outdata += ":"
            outdata += str(self.LateralThrusterRF.GetSpeed())
            outdata += "\n"
            self.serial.write(outdata.encode('utf-8'))
        if not self.secondSetTrade:
            outdata += str(self.VentralThrusterLB.name)
            outdata += ":"
            outdata += str(self.VentralThrusterLB.GetSpeed())
            outdata += ","
            outdata += str(self.VentralThrusterLF.name)
            outdata += ":"
            outdata += str(self.VentralThrusterLF.GetSpeed())
            outdata += ","
            outdata += str(self.VentralThrusterRB.name)
            outdata += ":"
            outdata += str(self.VentralThrusterRB.GetSpeed())
            outdata += ","
            outdata += str(self.VentralThrusterRF.name)
            outdata += ":"
            outdata += str(self.VentralThrusterRF.GetSpeed())
            outdata += "\n"
            self.serial.write(outdata.encode('utf-8'))
        self.secondSetTrade = not self.secondSetTrade

    def CheckIfPositionDone(self, threshold=3, timethreshold=5):
        self.PositionRunning = True
        if (abs(self.Gyro_hive.getNorth() - self.NorthOffset) < threshold) and (
                abs(self.Gyro_hive.getEast() - self.EastOffset) < threshold) and (
                abs(self.Gyro_hive.getDown() - self.DownOffset) < threshold):
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within position threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.PositionRunning = False
        else:
            self.InitialTime = time.perf_counter()
        return self.PositionRunning

    def CalculatePID(self):
        pass

    def BasicDirectionPower(self, index, power=15):

    def UpdateThrusters(self):
        # ex: self.LateralThrusterLB.SetSpeed(self.LateralPowerLB)

    def UpdateThrustersPID(self):
        # ex: self.LateralThrusterLB.SetSpeedPID(self.LateralPowerLB, yawpid=self.Gyro_hive.getYawPID())

    def UpdateGyro(self):
        if self.UsingGyro:
            self.Gyro_hive.UpdateGyro()
            # print(self.Gyro.getGyro())
            self.Gyro_hive.CalculateError(self.YawOffset,
                                          self.PitchOffset,
                                          self.RollOffset,
                                          self.NorthOffset,
                                          self.EastOffset,
                                          self.DownOffset)
            self.Gyro_hive.PID()

    def BrakeAllThrusters(self):
        # horizontal
        self.UpdateThrusters()


    # end vehicle connection and runtime after mission completion or a major fucky wucky
    def Terminate(self):

# dedicated class to driving a specific motor
class MotorDriver:
    def __init__(self, name):
        self.name = name
        self.speed = 0

    def SetSpeed(self, speed):  # speed is a value between -100 and 100
        if speed > MAX_THROTTLE:
            speed = MAX_THROTTLE
        elif speed < -MAX_THROTTLE:
            speed = -MAX_THROTTLE
        self.speed = MapToPWM(speed)

    #  sets speed of thruster and incorporates the addition of pwm variables
    def SetSpeedPID(self, speed, x=0.0, y=0.0, z=0.0):
        self.speed = float(float(speed) + float(x) + float(y) + float(z))
        if self.speed > MAX_THROTTLE:
            self.speed = MAX_THROTTLE
        elif self.speed < -MAX_THROTTLE:
            self.speed = -MAX_THROTTLE
        self.speed = MapToPWM(self.speed)

    # returns speed
    def GetSpeed(self):
        return self.speed


def MapToPWM(x):
    in_min = -100.0
    in_max = 100.0
    out_min = 1100
    out_max = 1900
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
