#!python3
# Author: Theodor Giles
# Created: 11/22/20
# Last Edited 5/19/22
# Description:
# This node manages the commands/movement/physical
# control of the RoboSub V2, 2020-21

import time
import random
import math
import imu_ard
import pinger
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

MAX_THROTTLE = 40

GENERAL_THROTTLE = 17.5


class NavigationCommander:

    # initialize everything to supposed starting position
    def __init__(self, usingarduino=False,
                 usingvision=False,
                 usinggyro=False,
                 usingsim=False,
                 usingping=False,
                 resetheadingoncmd=False):
        # setting up board serial port
        self.UsingArduino = usingarduino

        if self.UsingArduino:
            from arduino_commander import ArduinoCommander
            self.ArduinoCommander = ArduinoCommander()
            print("MovementCommander is using Arduino, wait 7 for thruster arm...")
            # thruster hardpoint classes
            # 'ventral' are the central, vertically oriented thrusters
            # for roll/pitch and ascent/descent
            self.Thruster_VentralLB = ThrusterDriver("LB")  # left back
            self.Thruster_VentralLF = ThrusterDriver("LF")  # left front
            self.Thruster_VentralRB = ThrusterDriver("RB")  # right back
            self.Thruster_VentralRF = ThrusterDriver("RF")  # right front
            # 'lateral' are the outer, 45 deg. oriented thrusters for
            # yaw/turning and strafe movement
            self.Thruster_LateralBL = ThrusterDriver("BL")  # back left
            self.Thruster_LateralBR = ThrusterDriver("BR")  # back right
            self.Thruster_LateralFL = ThrusterDriver("FL")  # front left !
            self.Thruster_LateralFR = ThrusterDriver("FR")  # front right !
            # power values to set to the thruster hardpoints
            # horizontally oriented
            self.LateralPowerBL = 0
            self.LateralPowerFL = 0
            self.LateralPowerBR = 0
            self.LateralPowerFR = 0
            # vertically oriented
            self.VentralPowerLB = 0
            self.VentralPowerRB = 0
            self.VentralPowerRF = 0
            self.VentralPowerLF = 0

            # horizontally oriented
            self.LateralPowerBL_offset = -1
            self.LateralPowerFL_offset = -1
            self.LateralPowerBR_offset = -1
            self.LateralPowerFR_offset = -1
            # vertically oriented
            self.VentralPowerLB_offset = -2
            self.VentralPowerRB_offset = -2
            self.VentralPowerRF_offset = -2
            self.VentralPowerLF_offset = -2

            self.BrakeAllThrusters()
            time.sleep(7)
        else:
            print("MovementCommander is not using Arduino...")
        self.UsingGyro = usinggyro
        # arduino. omitting for now, in favor of pure rpi gyro reads
        # self.serial = serial.Serial('/dev/ttyAMA0', 115200)
        if self.UsingGyro:
            # print("Sending IMU")
            # self.SendToArduino("IMU")
            # self.JY62_1_IMU = imu_rpi.JY62('/dev/ttyUSB0', 0)
            # self.JY62_2_IMU = imu_rpi.JY62('/dev/ttyUSB1', 1)
            self.ArdIMU = imu_ard.ArduinoIMU(self.ArduinoCommander.getSerial())

        else:
            print("Sending NOIMU")
            # self.SendToArduino("NOIMU")
        if usingping:
            print("Using Pinger")
            self.Pinger = pinger.Pinger()
        if resetheadingoncmd:
            self.YawOffset = self.ArdIMU.StartingAngle[0]
            self.ResetHeadingOnCMD = resetheadingoncmd
        else:
            self.YawOffset = 0

        self.YawLocked = False
        self.PitchLocked = False
        self.RollLocked = False

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
            # from python.experimental.advanced_telemetry import Telemetry
            # self.TelemetrySim = Telemetry()
            print("MovementCommander is using Telemetry...")
        else:
            print("MovementCommander is not using Telemetry...")

        # initialize thruster values to brake (self.PowerXX set to 0^)
        # self.UpdateThrusters()

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
            "placeholder",
            "red_buoy",
            "blue_buoy",
            "green_buoy",
            "orange_buoy",
            "gate",
            "Cesar"]
        self.TargetList = []
        self.scanned_target_format = {
            "ID": "N/A",
            "TARGET CLASS": self.POSSIBLE_TARGETS[0],
            "LOCATION": (0.0, 0.0, 0.0),
            "ORIENTATION": (0.0, 0.0, 0.0),
            "IMPORTANCE": 1024
        }
        print("MovementCommander initialized...")
        self.StartingTime = time.perf_counter()

    def GyroTesting(self):
        # self.IMU.updateGyro()
        # print("Angle: ", self.IMU.getGyro())
        # self.JY62_1_IMU.updateGyro()
        # self.JY62_2_IMU.updateGyro()
        # gyro1 = self.JY62_1_IMU.getGyro()
        # gyro2 = self.JY62_2_IMU.getGyro()
        # x1 = "{:.4f}".format(gyro1[0])
        # y1 = "{:.4f}".format(gyro1[1])
        # z1 = "{:.4f}".format(gyro1[2])
        # x2 = "{:.4f}".format(gyro2[0])
        # y2 = "{:.4f}".format(gyro2[1])
        # z2 = "{:.4f}".format(gyro2[2])
        # print("IMU 1 Angle: ", x1, y1, z1)
        # print("IMU 2 Angle: ", x2, y2, z2)
        # self.BrakeAllThrusters()
        self.ArdIMU.UpdateAngle()
        print("Averaged Angle: ", self.ArdIMU.getAngle())
        # print
        self.ArdIMU.CalculateError(self.YawOffset, self.PitchOffset, self.RollOffset)
        self.ArdIMU.PID()
        # self.ArdIMU.UpdateFrontAngle()
        # print("Front angle, corrected: ", self.ArdIMU.getCorrectedFrontAngle())

        # self.ArdIMU.UpdateRearAngle()
        # print("Rear angle, corrected: ", self.ArdIMU.getCorrectedRearAngle())

        # print("Yaw PID: ", self.ArdIMU.getYawPID())

        # print("Yaw P: ", self.ArdIMU.Yaw_P)
        # print("Yaw I: ", self.ArdIMU.Yaw_I)
        # print("Yaw D: ", self.ArdIMU.Yaw_D)

        print("Yaw Error_Sum: ", self.ArdIMU.Error_Sum[0][0])
        # print("Pitch PID: ", self.ArdIMU.getPitchPID())
        # print("Roll PID: ", self.ArdIMU.getRollPID())
        pass

    def ResetGyro(self):
        # self.JY62_1_IMU.resetGyro()
        # self.JY62_2_IMU.resetGyro()
        pass

    def ArduinoTesting(self):
        # self.ArduinoCommander.CommunicateAllThrusters(100, 40, 40, 100, -25, 100, -25, 100)
        self.UpdateThrustersGyroPID()
        # time.sleep(.1)
        pass

    def BasicDriverControl(self):
        self.BrakeAllThrusters()
        DrivingWithControl = True
        print("Offsets, Yaw:Pitch:Roll - ",
              self.YawOffset, ":",
              self.PitchOffset, ":",
              self.RollOffset)
        print("Driver Control!!")
        while DrivingWithControl:
            DriveCommand = remote_control.get_wasdqerv_directional()
            self.BasicDirectionPower(DriveCommand)
            self.InitialTime = time.perf_counter()
            DrivingWithControl = DriveCommand != -2
            while self.CheckIfGyroDone(threshold=10, timethreshold=5):
                # self.ArdIMU.UpdateAngle()
                # print("Averaged Angle: ", self.ArdIMU.getAngle())
                # self.ArdIMU.CalculateError(self.YawOffset, self.PitchOffset, self.RollOffset)
                # self.ArdIMU.PID()
                # self.UpdateGyro()
                print("FL: ", self.Thruster_LateralFL.getSpeed(), "FR: ", self.Thruster_LateralFR.getSpeed())
                print("Yaw PID: ", self.ArdIMU.getYawPID())
                self.UpdateThrustersGyroPID()
            # DrivingWithControl = self.ControlIndividualMotor()
            # self.TradeWithArduino()

    def BasicWithTime(self):
        DrivingWithTime = True
        if self.SuppCommand == "":
            print("SuppCommand empty, defaulting to run for 5 seconds...")
            self.SuppCommand = "10"
        while DrivingWithTime:
            DrivingWithTime = (time.perf_counter() - self.InitialTime) < int(self.SuppCommand)
            # print("Time: ", time.perf_counter() - self.InitialTime)
            self.BasicDirectionPower(self.CommandIndex)

    def BasicLinear(self):
        DrivingWithTime = True
        if self.SuppCommand == "":
            print("SuppCommand empty, defaulting to run for 5 seconds...")
            self.SuppCommand = "10"
        while DrivingWithTime:
            DrivingWithTime = (time.perf_counter() - self.InitialTime) < int(self.SuppCommand)
            # print("Yaw, Roll, Pitch error: ", self.IMU.Yaw_PID, )
            # print("Time: ", time.perf_counter() - self.InitialTime)
            self.BasicDirectionPower(self.CommandIndex)

    def setDestination(self, north, east, down):
        self.NorthOffset = north
        self.EastOffset = east
        self.DownOffset = down

    def BasicVectoring(self):  # 'vector' should be a 3-integer array
        targeting = True
        navigating = True
        i = 0
        self.StoreCommandGyroOffsets()
        while targeting:
            self.UpdateThrustersGyroPID()
            self.CheckIfGyroDone(threshold=10, timethreshold=3)
            targeting = self.GyroRunning
        print("TARGETED VECTOR.")
        while navigating:
            self.UpdateThrustersGyroPID()
            navigating = self.CheckIfPositionDone(threshold=10, timethreshold=3)
            navigating = self.PositionRunning
        print("ARRIVED TO VECTOR.")

    def BasicVectoring(self, yaw, pitch, roll):
        targeting = True
        navigating = True
        i = 0
        self.YawOffset = yaw
        self.PitchOffset = pitch
        self.RollOffset = roll
        while targeting:
            targeting = self.CheckIfGyroDone(threshold=10, timethreshold=3)
            self.UpdateGyro()
            self.UpdateThrustersGyroPID()
            self.ArduinoCommander.CommunicateAllThrusters(100, 40, 40, 100, -25, 100, -25, 100)
        print("TARGETED VECTOR.")
        while navigating:
            navigating = self.CheckIfPositionDone(threshold=10, timethreshold=3)
            self.ArduinoCommander.CommunicateAllThrusters(100, 40, 40, 100, -25, 100, -25, 100)
        print("ARRIVED TO VECTOR.")

    def WaypointVectoring(self):  # arc vectoring
        self.StoreCommandGyroOffsets()
        self.Waypoints = []
        i = 1
        Waypointrange = int(((self.NorthOffset + self.EastOffset + self.DownOffset) / 3) / 15)
        waypointrange = 10
        for i in range(Waypointrange):
            if i > Waypointrange / 2:
                n = Waypointrange - i
            else:
                n = i
            self.Waypoints[i] = [((self.NorthOffset / i) + self.ArdIMU.Angle[NORTH]),
                                 ((self.EastOffset / i) + self.ArdIMU.Angle[EAST]),
                                 ((self.DownOffset / i) + self.ArdIMU.Angle[DOWN])]
            self.Waypoints[i][0] += ((self.Waypoints[i][0] * self.YawOffset / 90) / 5) * n
        print("VECTOR CALCULATED. USING ", len(self.Waypoints), " WAYPOINTS.")
        i = 1
        for i in range(Waypointrange):
            self.YawOffset = math.atan(self.Waypoints[i][1] / self.Waypoints[i][0])
            self.PitchOffset = math.atan(self.Waypoints[i][2] / ((self.Waypoints[i][0] + self.Waypoints[i][1]) / 2))
            self.CheckIfGyroDone()
            self.CheckIfPositionDone()
            while self.PositionRunning:
                self.CheckIfGyroDone()
                self.CheckIfPositionDone()
            print("WAYPOINT ", i, " REACHED.")
        print("WAYPOINTS PASSED, VECTOR REACHED.")

    def AdvancedVectoring(self):
        self.StoreCommandGyroOffsets()

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
        self.ArduinoCommander.SendToArduino("tc,")
        print("Calibrating and arming thrusters...")
        time.sleep(8)
        print("Calibrated and armed.")
        for command in commandlist:
            print("VectorCommander running: ", command)
            self.MainCommand = ""
            self.SuppCommand = ""
            j = 0
            for commandParsed in str(command).split('\n'):
                commandParsed.strip()
                print("commandParsed")
                if j == 0:
                    self.MainCommand = commandParsed
                if j == 1:
                    self.SuppCommand = commandParsed
                j = j + 1
            print("Main: ", self.MainCommand, ", Supplementary: ", self.SuppCommand)
            if self.MainCommand == "REMOTE":
                print("Driver Control With:")
                if self.SuppCommand == "KEYBOARD":
                    print("Keyboard!")
                    self.BasicDriverControl()
                else:
                    pass
            else:
                for basiccommand in self.BASIC_MOVEMENT_COMMANDS:
                    i = 0
                    if self.MainCommand == basiccommand:
                        self.InitialTime = time.perf_counter()
                        if self.UsingGyro:
                            print("Running basic command...")
                            self.BasicLinear()
                        else:
                            print("Running basic command with time due to no Gyro functionality...")
                            self.BasicWithTime()
                    i += 2
                    self.CommandIndex += 1
                self.CommandIndex = 0
                for advancedcommand in self.ADVANCED_MOVEMENT_COMMANDS:
                    i = 0
                    if self.MainCommand == advancedcommand:
                        self.InitialTime = time.perf_counter()
                        if self.UsingGyro:
                            print("Running advanced command...")
                            self.BasicVectoring()
                        else:
                            print("Can't run advanced command without Gyro functionality...")
                    i += 2
                    self.CommandIndex += 1
                self.CommandIndex = 0
                for targetcommand in self.TARGET_MOVEMENT_COMMANDS:
                    i = 0
                    if self.MainCommand == targetcommand:
                        self.InitialTime = time.perf_counter()

                        if self.UsingGyro:
                            print("Running target-based command...")
                            self.TargetMovement()
                        else:
                            print("Can't run target commands without Gyro and Vision functionality...")
                    i += 2
                    self.CommandIndex += 1
                self.CommandIndex = 0
            # print("Ran into issue parsing commands...")
            # self.Terminate()

    def StoreCommandGyroOffsets(self):
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

    def StoreCommandPositionOffsets(self):
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
        self.YawLocked = (abs(self.ArdIMU.getYaw() + self.YawOffset) < threshold)
        self.PitchLocked = (abs(self.ArdIMU.getPitch() - self.PitchOffset) < threshold)
        self.RollLocked = (abs(self.ArdIMU.getRoll() - self.RollOffset) < threshold)
        print("Yaw diff: ", abs(self.ArdIMU.getYaw() + self.YawOffset))
        # if self.YawLocked and self.PitchLocked and self.RollLocked:
        if self.YawLocked:
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within gyro threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.GyroRunning = False
        else:
            # print("Gyro:", self.ArdIMU.getGyro())
            self.InitialTime = time.perf_counter()
        return self.GyroRunning

    def CheckIfPositionDone(self, threshold=3, timethreshold=5):
        self.PositionRunning = True
        self.NorthLocked = (abs(self.ArdIMU.getNorth() - self.NorthOffset) < threshold)
        self.EastLocked = (abs(self.ArdIMU.getEast() - self.EastOffset) < threshold)
        self.DownLocked = (abs(self.ArdIMU.getDown() - self.DownOffset) < threshold)
        if self.NorthLocked and self.EastLocked and self.NorthLocked:
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within position threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.PositionRunning = False
        else:
            self.InitialTime = time.perf_counter()
        return self.PositionRunning

    def ReturnScanDict(self):
        scanned_target = self.scanned_target_format
        self.Vision.getDistance()

        return self.scanned_target_format

    def ControlIndividualMotor(self):
        running = True
        motor, speed = remote_control.set_individual_motor()
        if motor == -1:
            running = False
        elif motor == 0:
            self.LateralPowerBL = 0
            self.LateralPowerFL = 0
            self.LateralPowerBR = 0
            self.LateralPowerFR = 0
            self.VentralPowerLB = 0
            self.VentralPowerLF = 0
            self.VentralPowerRB = 0
            self.VentralPowerRF = 0
        elif motor == 1:
            self.LateralPowerBL = speed
        elif motor == 2:
            self.LateralPowerFL = speed
        elif motor == 3:
            self.LateralPowerBR = speed
        elif motor == 4:
            self.LateralPowerFR = speed
        elif motor == 5:
            self.VentralPowerLB = speed
        elif motor == 6:
            self.VentralPowerLF = speed
        elif motor == 7:
            self.VentralPowerRB = speed
        elif motor == 8:
            self.VentralPowerRF = speed
        self.UpdateThrusters()
        return running

    def BasicDirectionPower(self, index, power=15):
        # print("Index: ", index)
        # index = index + 1
        if index != 0:
            if index == 1:
                # print("MOVING FORWARDS")
                self.LateralPowerBL = power
                self.LateralPowerFL = power
                self.LateralPowerBR = power
                self.LateralPowerFR = power
                self.VentralPowerLB = 0
                self.VentralPowerLF = 0
                self.VentralPowerRB = 0
                self.VentralPowerRF = 0
            elif index == 2:
                # print("STRAFING LEFT")
                self.LateralPowerBL = power
                self.LateralPowerFL = -power
                self.LateralPowerBR = -power
                self.LateralPowerFR = power
                self.VentralPowerLB = 0
                self.VentralPowerLF = 0
                self.VentralPowerRB = 0
                self.VentralPowerRF = 0
            elif index == 3:
                # print("REVERSING")
                self.LateralPowerBL = -power
                self.LateralPowerFL = -power
                self.LateralPowerBR = -power
                self.LateralPowerFR = -power
                self.VentralPowerLB = 0
                self.VentralPowerLF = 0
                self.VentralPowerRB = 0
                self.VentralPowerRF = 0
            elif index == 4:
                # print("STRAFING RIGHT")
                self.LateralPowerBL = -power
                self.LateralPowerFL = power
                self.LateralPowerBR = power
                self.LateralPowerFR = -power
                self.VentralPowerLB = 0
                self.VentralPowerLF = 0
                self.VentralPowerRB = 0
                self.VentralPowerRF = 0
            elif index == 5:
                # print("TURNING LEFT")
                self.LateralPowerBL = -power
                self.LateralPowerFL = -power
                self.LateralPowerBR = power
                self.LateralPowerFR = power
                self.VentralPowerLB = 0
                self.VentralPowerLF = 0
                self.VentralPowerRB = 0
                self.VentralPowerRF = 0
                self.YawOffset = self.YawOffset - 30
            elif index == 6:
                # print("TURNING RIGHT")
                self.LateralPowerBL = power
                self.LateralPowerFL = power
                self.LateralPowerBR = -power
                self.LateralPowerFR = -power
                self.VentralPowerLB = 0
                self.VentralPowerLF = 0
                self.VentralPowerRB = 0
                self.VentralPowerRF = 0
                self.YawOffset = self.YawOffset + 30
            elif index == 7:
                # print("ASCENDING")
                self.LateralPowerBL = 0
                self.LateralPowerFL = 0
                self.LateralPowerBR = 0
                self.LateralPowerFR = 0
                self.VentralPowerLB = power
                self.VentralPowerLF = power
                self.VentralPowerRB = power
                self.VentralPowerRF = power
            elif index == 8:
                # print("DESCENDING")
                self.LateralPowerBL = 0
                self.LateralPowerFL = 0
                self.LateralPowerBR = 0
                self.LateralPowerFR = 0
                self.VentralPowerLB = -power
                self.VentralPowerLF = -power
                self.VentralPowerRB = -power
                self.VentralPowerRF = -power
            elif index == -1:
                # print("PAUSING")
                self.LateralPowerBL = 0
                self.LateralPowerFL = 0
                self.LateralPowerBR = 0
                self.LateralPowerFR = 0

                self.VentralPowerLB = 0
                self.VentralPowerLF = 0
                self.VentralPowerRB = 0
                self.VentralPowerRF = 0
            elif index == -2:
                # print("STOPPING")
                self.LateralPowerBL = 0
                self.LateralPowerFL = 0
                self.LateralPowerBR = 0
                self.LateralPowerFR = 0
                self.VentralPowerLB = 0
                self.VentralPowerLF = 0
                self.VentralPowerRB = 0
                self.VentralPowerRF = 0
        # if self.UsingGyro:
        #     self.UpdateGyro()
        #     self.UpdateThrustersGyroPID()
        # else:
        #     self.UpdateThrusters()

    def UpdateGyro(self):
        if self.UsingGyro:
            self.ArdIMU.UpdateAngle()
            # print(self.Gyro.getGyro())
            self.ArdIMU.CalculateError(self.YawOffset,
                                       self.PitchOffset,
                                       self.RollOffset,
                                       self.NorthOffset,
                                       self.EastOffset,
                                       self.DownOffset)
            self.ArdIMU.PID()

    def UpdateSensors(self):
        self.UpdateGyro()
        self.Pinger.UpdateDistance()

    def UpdateThrusters(self):

        self.Thruster_VentralLB.setSpeed(self.VentralPowerLB)
        self.Thruster_VentralRB.setSpeed(self.VentralPowerRB)
        self.Thruster_VentralLF.setSpeed(self.VentralPowerLF)
        self.Thruster_VentralRF.setSpeed(self.VentralPowerRF)

        self.Thruster_LateralBL.setSpeed(self.LateralPowerBL)
        self.Thruster_LateralFL.setSpeed(self.LateralPowerFL)
        self.Thruster_LateralBR.setSpeed(self.LateralPowerBR)
        self.Thruster_LateralFR.setSpeed(self.LateralPowerFR)
        self.SendThrusterCommands()
        # time.sleep(.1)

    def UpdateThrustersGyroPID(self):
        # self.ArdIMU.CalculateError(self.YawOffset, self.PitchOffset, self.RollOffset,
        #                            self.NorthOffset, self.EastOffset, self.DownOffset)
        self.Thruster_VentralLB.setSpeedPID(self.VentralPowerLB,
                                            -self.ArdIMU.getPitchPID()
                                            - self.ArdIMU.getRollPID()
                                            + self.Pinger.getPID())
        self.Thruster_VentralRB.setSpeedPID(self.VentralPowerRB,
                                            -self.ArdIMU.getPitchPID()
                                            + self.ArdIMU.getRollPID()
                                            + self.Pinger.getPID())
        self.Thruster_VentralLF.setSpeedPID(self.VentralPowerLF,
                                            self.ArdIMU.getPitchPID()
                                            - self.ArdIMU.getRollPID()
                                            + self.Pinger.getPID())
        self.Thruster_VentralRF.setSpeedPID(self.VentralPowerRF,
                                            self.ArdIMU.getPitchPID()
                                            + self.ArdIMU.getRollPID()
                                            + self.Pinger.getPID())

        self.Thruster_LateralBL.setSpeedPID(self.LateralPowerBL,
                                            self.ArdIMU.getYawPID())
        self.Thruster_LateralFL.setSpeedPID(self.LateralPowerFL,
                                            self.ArdIMU.getYawPID())
        self.Thruster_LateralBR.setSpeedPID(self.LateralPowerBR,
                                            -self.ArdIMU.getYawPID())
        self.Thruster_LateralFR.setSpeedPID(self.LateralPowerFR,
                                            -self.ArdIMU.getYawPID())
        self.SendThrusterCommands()
        # time.sleep(.1)

    def UpdateThrustersVisionPID(self):

        self.Thruster_LateralBL.setSpeedPID(self.LateralPowerBL, xpid=self.Vision.getXPID())
        self.Thruster_LateralFL.setSpeedPID(self.LateralPowerFL, xpid=self.Vision.getXPID())
        self.Thruster_LateralBR.setSpeedPID(self.LateralPowerBR, xpid=-self.Vision.getXPID())
        self.Thruster_LateralFR.setSpeedPID(self.LateralPowerFR, xpid=-self.Vision.getXPID())

        self.Thruster_VentralLB.setSpeedPID(self.VentralPowerLB,
                                            ypid=self.Vision.getYPID())
        self.Thruster_VentralRB.setSpeedPID(self.VentralPowerRB,
                                            ypid=self.Vision.getYPID())
        self.Thruster_VentralLF.setSpeedPID(self.VentralPowerLF,
                                            ypid=-self.Vision.getYPID())
        self.Thruster_VentralRF.setSpeedPID(self.VentralPowerRF,
                                            ypid=-self.Vision.getYPID())
        self.SendThrusterCommands()

    def UpdateThrustersGyroVisionPID(self):
        self.ArdIMU.CalculateError(self.YawOffset, self.PitchOffset, self.RollOffset,
                                   self.NorthOffset, self.EastOffset, self.DownOffset)

        self.Thruster_VentralLB.setSpeedPID(self.VentralPowerLB,
                                            self.ArdIMU.getRollPID() +
                                            self.ArdIMU.getPitchPID())
        self.Thruster_VentralRB.setSpeedPID(self.VentralPowerRB,
                                            self.ArdIMU.getRollPID() +
                                            self.ArdIMU.getPitchPID())
        self.Thruster_VentralLF.setSpeedPID(self.VentralPowerLF,
                                            self.ArdIMU.getRollPID() +
                                            self.ArdIMU.getPitchPID())
        self.Thruster_VentralRF.setSpeedPID(self.VentralPowerRF,
                                            self.ArdIMU.getRollPID() +
                                            self.ArdIMU.getPitchPID())

        self.Thruster_LateralBL.setSpeedPID(self.LateralPowerBL,
                                            self.ArdIMU.getYawPID())
        self.Thruster_LateralFL.setSpeedPID(self.LateralPowerFL,
                                            self.ArdIMU.getYawPID())
        self.Thruster_LateralBR.setSpeedPID(self.LateralPowerBR,
                                            -self.ArdIMU.getYawPID())
        self.Thruster_LateralFR.setSpeedPID(self.LateralPowerFR,
                                            -self.ArdIMU.getYawPID())
        self.SendThrusterCommands()
        # time.sleep(.1)

    def SendThrusterCommands(self):
        self.ArduinoCommander.CommunicateAllThrusters(self.Thruster_LateralBL.getSpeed() + self.LateralPowerBL_offset,
                                                      self.Thruster_LateralFL.getSpeed() + self.LateralPowerFL_offset,
                                                      self.Thruster_LateralBR.getSpeed() + self.LateralPowerBR_offset,
                                                      self.Thruster_LateralFR.getSpeed() + self.LateralPowerFR_offset,
                                                      self.Thruster_VentralLB.getSpeed() + self.VentralPowerLB_offset,
                                                      self.Thruster_VentralLF.getSpeed() + self.VentralPowerLF_offset,
                                                      self.Thruster_VentralRB.getSpeed() + self.VentralPowerRB_offset,
                                                      self.Thruster_VentralRF.getSpeed() + self.VentralPowerRF_offset)

    def BrakeAllThrusters(self):
        # horizontal
        self.LateralPowerBL = 0
        self.LateralPowerFL = 0
        self.LateralPowerBR = 0
        self.LateralPowerFR = 0
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
        print("Disarming Thrusters. Wait 6...")
        self.BrakeAllThrusters()
        time.sleep(6)
        if self.UsingArduino:
            print("Killing Arduino. Wait 1...")
            # self.ArduinoCommander.SendToArduino("STOP")
            time.sleep(1)
            self.ArduinoCommander.serial.close()
        if self.UsingVision:
            print("Killing Vision. Wait 1...")
            time.sleep(1)
            self.Vision.Terminate()
        print("Killing RPi. Wait 1...")
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
        self.speed = round(self.speed, 3)
        self.speed = speed

    #  sets speed of thruster and incorporates the addition of pwm variables
    def setSpeedPID(self, speed, pid):
        self.speed = float(float(speed) + float(pid))
        if self.speed > MAX_THROTTLE:
            self.speed = MAX_THROTTLE
        elif self.speed < -MAX_THROTTLE:
            self.speed = -MAX_THROTTLE
        self.speed = round(self.speed, 3)
        self.speed = self.speed

    # returns speed
    def getSpeed(self):
        return self.speed


def MapToPWM(x):
    in_min = -100.0
    in_max = 100.0
    out_min = 1100
    out_max = 1900
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def MapFromPWM(x):
    in_min = 1100
    in_max = 1900
    out_min = -100.0
    out_max = 100
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
