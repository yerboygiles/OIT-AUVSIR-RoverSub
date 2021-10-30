#!python3
# Author: Theodor Giles
# Created: 11/22/20
# Last Edited 7/29/21
# Description:
# This node manages the commands/movement/physical
# control of the RoboSub V2, 2020-21

import time
import random
import math
import serial
import imu_ard_data
import vision_v1
import remote_control

# from threading import Thread

GYRO: int = 0
POSITION: int = 1
YAW: int = 0
PITCH: int = 1
ROLL: int = 2
NORTH: int = 0
EAST: int = 1
DOWN: int = 2
# ROBOSUB
A_TARGET = 1
A_POSITION = 2
A_GYRO = 3

MAX_THROTTLE = 15

GENERAL_THROTTLE = 17.5


class NavigationCommander:

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
            self.IMU = imu_ard_data.WT61P(self.serial)

        else:
            print("Sending NOIMU")
            self.SendToArduino("NOIMU")

        if resetheadingoncmd:
            self.YawOffset = self.IMU.StartingGyro[0]
            self.ResetHeadingOnCMD = resetheadingoncmd

        self.YawLocked = False
        self.PitchLocked = False
        self.RollLocked = False

        self.YawOffset = 0
        self.PitchOffset = 0
        self.RollOffset = 0

        self.NorthLocked = False
        self.PitchLocked = False
        self.RollLocked = False

        self.EastOffset = 0
        self.NorthOffset = 0
        self.DownOffset = 0

        self.UsingVision = usingvision
        self.UsingSim = usingsim
        if self.UsingVision:
            # import Theos_Really_Good_Detection_Script as obj_det
            # self.VisionAI = obj_det.Detector("TensorFlow_Graph/Tflite", False)
            # print("MovementCommander is using Vision AI...")
            self.Vision = vision_v1.vision()
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
        self.Thruster_VentralLB = ThrusterDriver("LB")  # left back
        self.Thruster_VentralLF = ThrusterDriver("LF")  # left front
        self.Thruster_VentralRB = ThrusterDriver("RB")  # right back
        self.Thruster_VentralRF = ThrusterDriver("RF")  # right front
        self.Thruster_LateralLB = ThrusterDriver("BL")  # back left
        self.Thruster_LateralRB = ThrusterDriver("BR")  # back right
        self.Thruster_LateralLF = ThrusterDriver("FL")  # front left !
        self.Thruster_LateralRF = ThrusterDriver("FR")  # front right !
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
            "STRAFE LEFT",
            "BACKWARDS",
            "STRAFE RIGHT",
            "TURN LEFT",
            "TURN RIGHT",
            "ASCEND",
            "DESCEND",
            "PAUSE",
        ]
        self.ADVANCED_MOVEMENT_COMMANDS = [
            "LOG START POINT",
            "RETURN TO START",
            "GYRO TO",
            "POSITION TO",
            "SCAN FOR TARGET"
        ]
        self.TARGET_MOVEMENT_COMMANDS = [
            "MOVE TO",
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
        self.StartingTime = time.perf_counter()

    def BasicDriverControl(self):
        DrivingWithControl = True
        print("Driver Control!!")
        while DrivingWithControl:
            DriveCommand = remote_control.get_wasdqerv_directional()
            self.BasicDirectionPower(DriveCommand)
            DrivingWithControl = DriveCommand is not -2
            self.TradeWithArduino()

    def BasicWithTime(self):
        DrivingWithTime = True
        while DrivingWithTime:
            DrivingWithTime = (time.perf_counter() - self.InitialTime) < int(self.SuppCommand)
            self.BasicDirectionPower(self.CommandIndex)

    def BasicLinear(self):
        pass

    def BasicVectoring(self):
        Vectoring = True
        i = 0
        print("Supplemental: ", self.SuppCommand)
        for SuppParse in str(self.SuppCommand).split(':'):
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

    def WaypointVectoring(self):
        self.StoreGyroOffsets()
        self.Waypoints = []
        i = 1
        Waypointrange = int(((self.NorthOffset+self.EastOffset+self.DownOffset)/3)/15)
        waypointrange = 10
        for i in range(Waypointrange):
            self.Waypoints[i] = [((self.NorthOffset / i) + self.IMU.Offsets[NORTH]),
                            ((self.EastOffset / i) + self.IMU.Offsets[EAST]),
                            ((self.DownOffset / i) + self.IMU.Offsets[DOWN])]
            n = Waypointrange - i if i > Waypointrange/2 else n = i
            self.Waypoints[i][0] += ((self.Waypoints[i][0] * self.YawOffset/90)/5)*n
        i = 1
        for i in range(Waypointrange):
            self.YawOffset = math.atan(self.Waypoints[i][1]/self.Waypoints[i][0])
            self.PitchOffset = math.atan(self.Waypoints[i][2]/((self.Waypoints[i][0]+self.Waypoints[i][1])/2))
            self.CheckIfGyroDone()
            self.CheckIfPositionDone()
            while self.PositionRunning:
                self.CheckIfGyroDone()
                self.CheckIfPositionDone()
        print("Completed Waypoint Series")
    def AdvancedVectoring(self):
        self.StoreGyroOffsets()

    def TargetMovement(self):
        print("Scanning for target...")
        # while self.SearchAndLockTarget(self.SuppCommand):
        #     pass
        engaging = False
        ramtime = 0
        while engaging:
            self.Vision.StereoTarget(False)
            # 0- "MOVE TO"
            if self.CommandIndex == 0:
                if self.Vision.getDistance() < int(self.SuppCommand):
                    engaging = False
            # 1- "RAM",
            elif self.CommandIndex == 1:
                if self.Vision.getDistance() < int(self.SuppCommand):
                    self.BasicDirectionPower(1)
                    if time.perf_counter() - ramtime > (int(self.SuppCommand) / 4):
                        engaging = False
                else:
                    ramtime = time.perf_counter()
            # 2- "FIRE AT"
            # 3- "FOLLOW"
            self.UpdateThrustersVisionPID()

    def SearchAndLockTarget(self, target):
        scanstate = False
        state1_timer = 0
        state2_timer = 0
        confidence_timer: float = 0
        self.TargetLocked = False
        if not self.Vision.seesTargetColorMask(target):
            self.MovingToConfidence = False
            if scanstate:  # pause and look
                state1_timer = time.perf_counter()
                self.BasicDirectionPower(-2)
                if state1_timer - state2_timer > 5:
                    scanstate = False
            else:  # increment and look, 6 = right, 5 = left
                state2_timer = time.perf_counter()
                self.BasicDirectionPower(6)
                if state2_timer - state1_timer > 3:
                    scanstate = True
        else:
            if not self.MovingToConfidence:
                confidence_timer = time.perf_counter()
                self.MovingToConfidence = True
            else:
                if time.perf_counter() - confidence_timer > 5:
                    return False
                self.BasicDirectionPower(-2)
        self.UpdateThrusters()
        return True

    # Concept code, basically for checking if the Sub has already seen the detected object.
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

    def receiveCommands(self, commandlist):
        # going through commands in parsed list
        self.CommandIndex = 0
        # tell arduino to arm motors
        self.SendToArduino("STOP")
        print("Stopping arduino... Wait 3.")
        time.sleep(3)
        self.SendToArduino("START")
        print("Starting arduino... Wait 3.")
        time.sleep(3)
        self.SendToArduino("MAXPOWER:20")
        print("Sending settings... Wait 3.")
        time.sleep(3)
        try:
            for command in commandlist:
                print("VectorCommander running: ", command)
                self.MainCommand = ""
                self.SuppCommand = ""
                j = 0
                for commandParsed in str(command).split(','):
                    commandParsed.strip()
                    if j == 0:
                        self.MainCommand = commandParsed
                    if j == 1:
                        self.SuppCommand = commandParsed
                    j = j + 1
                print("Main: ", self.MainCommand, ", Supplementary: ", self.SuppCommand)
                if self.MainCommand == "REMOTE":
                    print("Driver Control With:")
                    self.BasicDriverControl()
                    if self.SuppCommand == "KEYBOARD":
                        print("Keyboard!")
                    else:
                        pass
                else:
                    print("Searching basic movement...")
                    for basiccommand in self.BASIC_MOVEMENT_COMMANDS:
                        i = 0
                        if self.MainCommand == basiccommand:
                            self.InitialTime = time.perf_counter()
                            if self.UsingGyro:
                                self.BasicLinear()
                            else:
                                self.BasicWithTime()
                        i += 2
                        self.CommandIndex += 1
                    self.CommandIndex = 0
                    print("Searching advanced movement...")
                    for advancedcommand in self.ADVANCED_MOVEMENT_COMMANDS:
                        i = 0
                        if self.MainCommand == advancedcommand:
                            self.InitialTime = time.perf_counter()
                            self.BasicVectoring()
                        i += 2
                        self.CommandIndex += 1
                    self.CommandIndex = 0
                    print("Searching target movement...")
                    for targetcommand in self.TARGET_MOVEMENT_COMMANDS:
                        i = 0
                        if self.MainCommand == targetcommand:
                            self.InitialTime = time.perf_counter()
                            self.TargetMovement()
                        i += 2
                        self.CommandIndex += 1
                    self.CommandIndex = 0
        except:
            self.Terminate()

    def StoreGyroOffsets(self):
        i = 0
        for SuppParse in str(self.SuppCommand).split(':'):
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

    def StorePositionOffsets(self):
        i = 0
        for SuppParse in str(self.SuppCommand).split(':'):
            print("SuppParse: ", SuppParse)
            if i == 0:
                self.NorthOffset = float(SuppParse)
                # print("YawOffset: ", self.YawOffset)
            if i == 1:
                self.EastOffset = float(SuppParse)
            if i == 2:
                self.DownOffset = float(SuppParse)
            if i > 2:
                break
            i = i + 1

    def CheckIfGyroDone(self, threshold=15, timethreshold=5):
        # if(self.Gyro.getYaw() < 0):
        self.GyroRunning = True
        integer = 0
        self.UpdateGyro()
        self.YawLocked = (abs(self.IMU.getYaw() - abs(self.YawOffset)) < threshold)
        self.PitchLocked = (abs(self.IMU.getPitch() - abs(self.PitchOffset)) < threshold)
        self.RollLocked = (abs(self.IMU.getRoll() - abs(self.RollOffset)) < threshold)

        if self.YawLocked and self.PitchLocked and self.RollLocked:
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within gyro threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.GyroRunning = False
        else:
            print("Gyro:", self.IMU.getGyro())
            self.InitialTime = time.perf_counter()

    def SendToArduino(self, whattosend):
        self.serial.write(whattosend.encode('utf-8'))

    def TradeWithArduino(self):
        self.UpdateThrusters()
        outdata = ""
        if self.secondSetTrade:
            outdata += str(self.Thruster_LateralLB.name)
            outdata += ":"
            outdata += str(self.Thruster_LateralLB.getSpeed())
            outdata += ","
            outdata += str(self.Thruster_LateralRB.name)
            outdata += ":"
            outdata += str(self.Thruster_LateralRB.getSpeed())
            outdata += ","
            outdata += str(self.Thruster_LateralLF.name)
            outdata += ":"
            outdata += str(self.Thruster_LateralLF.getSpeed())
            outdata += ","
            outdata += str(self.Thruster_LateralRF.name)
            outdata += ":"
            outdata += str(self.Thruster_LateralRF.getSpeed())
            outdata += "\n"
            self.serial.write(outdata.encode('utf-8'))
        if not self.secondSetTrade:
            outdata += str(self.Thruster_VentralLB.name)
            outdata += ":"
            outdata += str(self.Thruster_VentralLB.getSpeed())
            outdata += ","
            outdata += str(self.Thruster_VentralLF.name)
            outdata += ":"
            outdata += str(self.Thruster_VentralLF.getSpeed())
            outdata += ","
            outdata += str(self.Thruster_VentralRB.name)
            outdata += ":"
            outdata += str(self.Thruster_VentralRB.getSpeed())
            outdata += ","
            outdata += str(self.Thruster_VentralRF.name)
            outdata += ":"
            outdata += str(self.Thruster_VentralRF.getSpeed())
            outdata += "\n"
            self.serial.write(outdata.encode('utf-8'))
        self.secondSetTrade = not self.secondSetTrade

    def CheckIfPositionDone(self, threshold=3, timethreshold=5):
        self.PositionRunning = True
        self.NorthLocked = (abs(self.IMU.getNorth() - self.NorthOffset) < threshold)
        self.EastLocked = (abs(self.IMU.getEast() - self.EastOffset) < threshold)
        self.DownLocked = (abs(self.IMU.getDown() - self.DownOffset) < threshold)
        if self.NorthLocked and self.EastLocked and self.NorthLocked:
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
        if index != 0:
            if index == 1:
                print("MOVING FORWARDS")
                self.LateralPowerLB = power
                self.LateralPowerLF = power
                self.LateralPowerRB = power
                self.LateralPowerRF = power
            elif index == 2:
                print("STRAFING LEFT")
                self.LateralPowerLB = power
                self.LateralPowerLF = -power
                self.LateralPowerRB = -power
                self.LateralPowerRF = power
            elif index == 3:
                print("REVERSING")
                self.LateralPowerLB = -power
                self.LateralPowerLF = -power
                self.LateralPowerRB = -power
                self.LateralPowerRF = -power
            elif index == 4:
                print("STRAFING RIGHT")
                self.LateralPowerLB = -power
                self.LateralPowerLF = power
                self.LateralPowerRB = power
                self.LateralPowerRF = -power
            elif index == 5:
                print("TURNING LEFT")
                self.LateralPowerLB = -power
                self.LateralPowerLF = -power
                self.LateralPowerRB = power
                self.LateralPowerRF = power
            elif index == 6:
                print("TURNING RIGHT")
                self.LateralPowerLB = power
                self.LateralPowerLF = power
                self.LateralPowerRB = -power
                self.LateralPowerRF = -power
            elif index == 7:
                print("ASCENDING")
                self.LateralPowerLB = power
                self.LateralPowerLF = power
                self.LateralPowerRB = -power
                self.LateralPowerRF = -power
            elif index == 8:
                print("DESCENDING")
                self.LateralPowerLB = power
                self.LateralPowerLF = power
                self.LateralPowerRB = -power
                self.LateralPowerRF = -power
            elif index == -1:
                print("PAUSING")
                self.LateralPowerLB = 0
                self.LateralPowerLF = 0
                self.LateralPowerRB = 0
                self.LateralPowerRF = 0
            elif index == -2:
                print("STOPPING")
                self.LateralPowerLB = 0
                self.LateralPowerLF = 0
                self.LateralPowerRB = 0
                self.LateralPowerRF = 0

    def UpdateThrusters(self):
        self.Thruster_LateralLB.setSpeed(self.LateralPowerLB)
        self.Thruster_LateralLF.setSpeed(self.LateralPowerLF)
        self.Thruster_LateralRB.setSpeed(self.LateralPowerRB)
        self.Thruster_LateralRF.setSpeed(self.LateralPowerRF)

        self.Thruster_VentralLB.setSpeed(self.VentralPowerLB)
        self.Thruster_VentralRB.setSpeed(self.VentralPowerRB)
        self.Thruster_VentralLF.setSpeed(self.VentralPowerLF)
        self.Thruster_VentralRF.setSpeed(self.VentralPowerRF)

    def UpdateThrustersGyroVisionPID(self):
        self.Thruster_LateralLB.setSpeedPID(self.LateralPowerLB,
                                            xpid=self.IMU.getYawPID() + self.Vision.getXPID())
        self.Thruster_LateralLF.setSpeedPID(self.LateralPowerLF,
                                            xpid=self.IMU.getYawPID() + self.Vision.getXPID())
        self.Thruster_LateralRB.setSpeedPID(self.LateralPowerRB,
                                            xpid=-self.IMU.getYawPID() - self.Vision.getXPID())
        self.Thruster_LateralRF.setSpeedPID(self.LateralPowerRF,
                                            xpid=-self.IMU.getYawPID() - self.Vision.getXPID())

        self.Thruster_VentralLB.setSpeedPID(self.VentralPowerLB,
                                            zpid=self.IMU.getRollPID(),
                                            ypid=self.IMU.getPitchPID() + self.Vision.getYPID())
        self.Thruster_VentralRB.setSpeedPID(self.VentralPowerRB,
                                            zpid=-self.IMU.getRollPID(),
                                            ypid=self.IMU.getPitchPID() + self.Vision.getYPID())
        self.Thruster_VentralLF.setSpeedPID(self.VentralPowerLF,
                                            zpid=-self.IMU.getRollPID(),
                                            ypid=-self.IMU.getPitchPID() - self.Vision.getYPID())
        self.Thruster_VentralRF.setSpeedPID(self.VentralPowerRF,
                                            zpid=self.IMU.getRollPID(),
                                            ypid=-self.IMU.getPitchPID() - self.Vision.getYPID())

    def UpdateThrustersGyroPID(self):
        self.Thruster_LateralLB.setSpeedPID(self.LateralPowerLB, xpid=self.IMU.getYawPID())
        self.Thruster_LateralLF.setSpeedPID(self.LateralPowerLF, xpid=self.IMU.getYawPID())
        self.Thruster_LateralRB.setSpeedPID(self.LateralPowerRB, xpid=-self.IMU.getYawPID())
        self.Thruster_LateralRF.setSpeedPID(self.LateralPowerRF, xpid=-self.IMU.getYawPID())

        self.Thruster_VentralLB.setSpeedPID(self.VentralPowerLB,
                                            zpid=self.IMU.getRollPID(),
                                            ypid=-self.IMU.getPitchPID())
        self.Thruster_VentralRB.setSpeedPID(self.VentralPowerRB,
                                            zpid=-self.IMU.getRollPID(),
                                            ypid=-self.IMU.getPitchPID())
        self.Thruster_VentralLF.setSpeedPID(self.VentralPowerLF,
                                            zpid=-self.IMU.getRollPID(),
                                            ypid=-self.IMU.getPitchPID())
        self.Thruster_VentralRF.setSpeedPID(self.VentralPowerRF,
                                            zpid=self.IMU.getRollPID(),
                                            ypid=-self.IMU.getPitchPID())

    def UpdateThrustersVisionPID(self):

        self.Thruster_LateralLB.setSpeedPID(self.LateralPowerLB, xpid=self.Vision.getXPID())
        self.Thruster_LateralLF.setSpeedPID(self.LateralPowerLF, xpid=self.Vision.getXPID())
        self.Thruster_LateralRB.setSpeedPID(self.LateralPowerRB, xpid=-self.Vision.getXPID())
        self.Thruster_LateralRF.setSpeedPID(self.LateralPowerRF, xpid=-self.Vision.getXPID())

        self.Thruster_VentralLB.setSpeedPID(self.VentralPowerLB,
                                            ypid=self.Vision.getYPID())
        self.Thruster_VentralRB.setSpeedPID(self.VentralPowerRB,
                                            ypid=self.Vision.getYPID())
        self.Thruster_VentralLF.setSpeedPID(self.VentralPowerLF,
                                            ypid=-self.Vision.getYPID())
        self.Thruster_VentralRF.setSpeedPID(self.VentralPowerRF,
                                            ypid=-self.Vision.getYPID())

    def UpdateGyro(self):
        if self.UsingGyro:
            self.IMU.UpdateGyro()
            # print(self.Gyro.getGyro())
            self.IMU.CalculateError(self.YawOffset,
                                    self.PitchOffset,
                                    self.RollOffset,
                                    self.NorthOffset,
                                    self.EastOffset,
                                    self.DownOffset)
            self.IMU.PID()

    def BrakeAllThrusters(self):
        # horizontal
        self.LateralPowerLB = 0
        self.LateralPowerLF = 0
        self.LateralPowerRB = 0
        self.LateralPowerRF = 0
        # vert
        self.VentralPowerLB = 0
        self.VentralPowerRB = 0
        self.VentralPowerRF = 0
        self.VentralPowerLF = 0

        self.UpdateThrusters()

    def printTelemetry(self):
        print("Ventral LB: ", self.Thruster_VentralLB.getSpeed())
        print("Ventral LF: ", self.Thruster_VentralLF.getSpeed())
        print("Ventral RB: ", self.Thruster_VentralRB.getSpeed())
        print("Ventral RF: ", self.Thruster_VentralRF.getSpeed())
        print("Lateral LB: ", self.Thruster_VentralLB.getSpeed())
        print("Lateral RB: ", self.Thruster_VentralRB.getSpeed())
        print("Lateral RF: ", self.Thruster_VentralRF.getSpeed())
        print("Lateral LF: ", self.Thruster_VentralLF.getSpeed())

        print("Vision Offset: ", self.Vision.getOffset())

        self.RunningTime = time.perf_counter() - self.StartingTime
        print("Running Time: ", self.RunningTime)

    # searches for target if cannot find it
    # def SearchForTarget(self, target, repositioning=False, distancethreshold=300):

    # ending vehicle connection and AI processing after mission completion or a major fucky wucky
    def Terminate(self):
        self.Thruster_VentralLB.setSpeed(0)
        self.Thruster_VentralLF.setSpeed(0)
        self.Thruster_VentralRB.setSpeed(0)
        self.Thruster_VentralRF.setSpeed(0)
        self.Thruster_LateralLB.setSpeed(0)
        self.Thruster_LateralRB.setSpeed(0)
        self.Thruster_LateralRF.setSpeed(0)
        self.Thruster_LateralLF.setSpeed(0)
        # self.UpdateThrusters()
        self.SendToArduino("STOP")
        time.sleep(1)
        if self.UsingVision:
            print("Killing Vision. Wait 1...")
            time.sleep(1)
            self.Vision.Terminate()
        print("Killing board. Wait 1...")
        time.sleep(1)


# dedicated class to driving a specific thruster
# has own PID, thruster, speed
class ThrusterDriver:
    def __init__(self, name):
        self.name = name
        self.speed = 0

    def setSpeed(self, speed):  # speed is a value between -100 and 100
        if speed > MAX_THROTTLE:
            speed = MAX_THROTTLE
        elif speed < -MAX_THROTTLE:
            speed = -MAX_THROTTLE
        self.speed = MapToPWM(speed)

    #  sets speed of thruster and incorporates the addition of pwm variables
    def setSpeedPID(self, speed, zpid=0.0, ypid=0.0, xpid=0.0):
        self.speed = float(float(speed) + float(zpid) + float(ypid) + float(xpid))
        if self.speed > MAX_THROTTLE:
            self.speed = MAX_THROTTLE
        elif self.speed < -MAX_THROTTLE:
            self.speed = -MAX_THROTTLE
        self.speed = MapToPWM(self.speed)

    # returns speed
    def getSpeed(self):
        return self.speed


def MapToPWM(x):
    in_min = -100.0
    in_max = 100.0
    out_min = 1100
    out_max = 1900
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
