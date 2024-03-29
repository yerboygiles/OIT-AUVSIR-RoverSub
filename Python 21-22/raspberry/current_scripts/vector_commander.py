#!python3
# Author: Theodor Giles, Colton Rhodes
# Created: 11/22/20
# Last Edited 7/23/22
# Description:
# This node manages the commands/movement/physical
# control of the RoboSub V2, 2020-21

import time
import random
import math
import imu_ard
import sonar
import vision_v2
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
    def __init__(self,
                 usingvision=False,
                 usinggyro=False,
                 usingsim=False,
                 usingsonar=False,
                 resetheadingoncmd=False):
        # setting up board serial port

        # self.writeout = open('telemetry_active.txt', 'w')

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

        self.UsingVision = usingvision
        self.UsingGyro = usinggyro

        if self.UsingGyro:
            print("Using Gyro")
            self.ArdIMU = imu_ard.ArduinoIMU(self.ArduinoCommander.getSerial())
        time.sleep(7)
        self.passedGate = False

        self.UsingSonar = usingsonar
        if self.UsingSonar:
            print("Using Pinger")
            self.Sonar = sonar.Sonar()
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
            self.Vision = vision_v2.vision()
        else:
            print("MovementCommander is not using Vision AI...")

        if self.UsingSim:
            # from python.experimental.advanced_telemetry import Telemetry
            # self.TelemetrySim = Telemetry()
            print("MovementCommander is using Simulation...")
        else:
            print("MovementCommander is not using Simulation...")

        # string list of movement commands, because I thought I'd make
        # the index number of each command streamlined with other
        # functions, but it seems a bit detrimental the more I work with
        # it.
        # advanced: these commands are much more complicated, will need to
        # develop pathing and a lot of vision/gyro/position integration
        self.BASIC_MOVEMENT_COMMANDS = [
            "STALL",
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
        self.BrakeAllThrusters()
        print("MovementCommander initialized...")
        self.StartingTime = time.perf_counter()

    def ZeroSonar(self):
        self.Sonar.setOffsetCurrent()

    def VisionTesting(self):

        self.Vision.process_image(searchingfor=2)  # 2 - buoys, 1 - gate
        print("Post-image process")
        self.Vision.CalculateError()
        print("Post-calculate")
        pass

    def GyroTesting(self):
        # self.ArdIMU.UpdatePosition()
        self.ArdIMU.UpdateAngle()
        self.ArdIMU.UpdateAcceleration()
        # self.ArdIMU.UpdatePosition()
        self.ArdIMU.CalculateError()
        self.ArdIMU.CalculatePID()

        pass

    def ResetGyro(self):
        # self.JY62_1_IMU.resetGyro()
        # self.JY62_2_IMU.resetGyro()
        pass

    def ArduinoTesting(self):
        # self.ArduinoCommander.CommunicateAllThrusters(100, 40, 40, 100, -25, 100, -25, 100)
        # self.UpdateThrusters_Gyro_PID()
        self.UpdateThrusters_Gyro_PID_NoCompass()
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
            while self.GyroLocking():
                # self.ArdIMU.UpdateAngle()
                # print("Averaged Angle: ", self.ArdIMU.getAngle())
                # self.ArdIMU.CalculateError(self.YawOffset, self.PitchOffset, self.RollOffset)
                # self.ArdIMU.PID()
                # self.UpdateGyro()
                print("FL: ", self.Thruster_LateralFL.getSpeed(), "FR: ", self.Thruster_LateralFR.getSpeed())
                print("Yaw PID: ", self.ArdIMU.getYawPID())
                self.setThrusters_GyroPID()
            # DrivingWithControl = self.ControlIndividualMotor()
            # self.TradeWithArduino()

    def BasicWithTime(self):
        self.InitialTime = time.perf_counter()
        DrivingWithTime = True
        print("Command index: ", self.CommandIndex)
        if self.SuppCommand == "":
            print("SuppCommand empty, defaulting to run for 5 seconds...")
            self.SuppCommand = "10"
        while DrivingWithTime:
            print("Time left: ", (time.perf_counter() - self.InitialTime))
            DrivingWithTime = (time.perf_counter() - self.InitialTime) < int(self.SuppCommand)
            print("FL Speed: ", self.Thruster_LateralFL.getSpeed())
            # print("Time: ", time.perf_counter() - self.InitialTime)
            self.BasicDirectionPower(self.CommandIndex)

    def BasicLinear(self):
        DrivingWithGyro = True
        if self.SuppCommand == "":
            print("SuppCommand empty, defaulting to run for 5 seconds...")
            self.SuppCommand = "x10"  # 'x10' go 10 measures in the x direction
        elif self.SuppCommand[0] is 'y':
            usedAngle = 1
        elif self.SuppCommand[0] is 'z':
            usedAngle = 2
        else:
            usedAngle = 0
        measure = int(self.SuppCommand[1:])
        while DrivingWithGyro:
            angles = self.ArdIMU.getAngle()
            DrivingWithGyro = (angles[usedAngle] - measure) > 0
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
        self.StoreCommandGyroOffsets()
        while targeting:
            self.ArdIMU.updateGyro()
            self.setThrusters_GyroPID()
            self.GyroLocking()
            targeting = self.gyro_locking
        print("TARGETED VECTOR.")
        # while navigating:
        #     self.UpdateThrusters_Gyro_PID()
        #     navigating = self.CheckIfPositionDone()
        # print("ARRIVED TO VECTOR.")

    # def BasicVectoring(self, yaw, pitch, roll):
    #     targeting = True
    #     navigating = True
    #     self.YawOffset = yaw
    #     self.PitchOffset = pitch
    #     self.RollOffset = roll
    #     while targeting:
    #         self.UpdateThrusters_Gyro_PID()
    #         targeting = self.CheckIfGyroDone()
    #     print("TARGETED VECTOR.")
    #     while navigating:
    #         targeting = self.CheckIfGyroDone()
    #         navigating = self.CheckIfPositionDone()
    #     print("ARRIVED TO VECTOR.")

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
            self.GyroLocking()
            self.CheckIfPositionDone()
            while self.PositionRunning:
                self.GyroLocking()
                self.CheckIfPositionDone()
            print("WAYPOINT ", i, " REACHED.")
        print("WAYPOINTS PASSED, VECTOR REACHED.")

    def AdvancedVectoring(self):
        self.StoreCommandGyroOffsets()

    def ApproachTarget(self):
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

    # def receiveCommands(self, commandlist):
    #     # going through commands in parsed list
    #     self.CommandIndex = 0
    #     # tell arduino to arm motors
    #     self.ArduinoCommander.SendToArduino("tc,")
    #     print("Calibrating and arming thrusters...")
    #     time.sleep(7)
    #     self.ArduinoCommander.SendToArduino("gfc,")
    #     print("Calibrating front IMU. Wait 5...")
    #     time.sleep(5)
    #     self.ArduinoCommander.SendToArduino("grc,")
    #     print("Calibrating rear IMU. Wait 5...")
    #     time.sleep(5)
    #     print("Calibrated and armed.")
    #
    #     for command in commandlist:
    #         print("VectorCommander running: ", command)
    #         self.MainCommand = ""
    #         self.SuppCommand = ""
    #         j = 0
    #         for commandParsed in str(command).split(','):
    #             commandParsed.strip()
    #             # print("commandParsed")
    #             if j == 0:
    #                 self.MainCommand = commandParsed
    #                 print(command, " main: ", self.MainCommand)
    #             if j == 1:
    #                 self.SuppCommand = commandParsed
    #                 print(command, " supp: ", self.SuppCommand)
    #             j = j + 1
    #         print("Main: ", self.MainCommand, ", Supplementary: ", self.SuppCommand)
    #         if self.MainCommand == "REMOTE":
    #             print("Driver Control With:")
    #             if self.SuppCommand == "KEYBOARD":
    #                 print("Keyboard!")
    #                 self.BasicDriverControl()
    #             else:
    #                 pass
    #         else:
    #             for basiccommand in self.BASIC_MOVEMENT_COMMANDS:
    #                 i = 0
    #                 if self.MainCommand == basiccommand:
    #                     self.InitialTime = time.perf_counter()
    #                     if self.UsingGyro:
    #                         print("Running basic command...")
    #                         # self.BasicLinear()
    #                         self.BasicWithTime()
    #                     else:
    #                         pass
    #                         print("Running N/A command with time due to no Gyro functionality...")
    #                 i += 2
    #                 self.CommandIndex += 1
    #             self.CommandIndex = 0
    #             for advancedcommand in self.ADVANCED_MOVEMENT_COMMANDS:
    #                 i = 0
    #                 if self.MainCommand == advancedcommand:
    #                     self.InitialTime = time.perf_counter()
    #                     self.StoreCommandGyroOffsets()
    #                     # self.StoreCommandPositionOffsets()
    #                     if self.UsingGyro:
    #                         print("Running advanced command...")
    #                         self.BasicVectoring()
    #                     else:
    #                         print("Can't run advanced command without Gyro functionality...")
    #                 i += 2
    #                 self.CommandIndex += 1
    #             self.CommandIndex = 0
    #             for targetcommand in self.TARGET_MOVEMENT_COMMANDS:
    #                 i = 0
    #                 if self.MainCommand == targetcommand:
    #                     self.InitialTime = time.perf_counter()
    #
    #                     if self.UsingGyro:
    #                         print("Running target-based command...")
    #                         self.ApproachTarget()
    #                     else:
    #                         print("Can't run target commands without Gyro and Vision functionality...")
    #                 i += 2
    #                 self.CommandIndex += 1
    #             self.CommandIndex = 0
    #         # print("Ran into issue parsing commands...")
    #         # self.Terminate()

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

    def setSonarOffset(self, slices=5, height=3):
        self.Sonar.setOffset((self.Sonar.getStartingDistance() / slices) * height)

    def SonarLocking(self, threshold=3, timethreshold=5):
        # if(self.Gyro.getYaw() < 0):
        self.sonar_locking = True
        integer = 0
        self.Sonar.update()
        # self.Sonar.setOffsetCurrent(self.Sonar.getDistance())
        self.HeightLocked = (abs(self.Sonar.getDistance() - self.Sonar.DistanceOffset < threshold))

        if self.HeightLocked:
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within  sonar height threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.sonar_locking = False
        else:
            # print("Gyro:", self.ArdIMU.getGyro())
            self.InitialTime = time.perf_counter()
        return self.SonarLocking

    def GyroLocking(self, threshold=3, timethreshold=5):
        self.gyro_locking = True
        self.UpdateGyro()
        self.YawLocked = (abs(self.ArdIMU.getYawPID()) < threshold)
        self.PitchLocked = (abs(self.ArdIMU.getPitchPID()) < threshold)
        self.RollLocked = (abs(self.ArdIMU.getRollPID()) < threshold)
        if self.YawLocked:
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within gyro threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.gyro_locking = False
        else:
            self.InitialTime = time.perf_counter()
        return self.gyro_locking

    def CheckIfPositionDone(self, threshold=5, timethreshold=5):
        self.PositionRunning = True
        self.NorthLocked = (abs(self.ArdIMU.getNorthPID()) < threshold)
        self.EastLocked = (abs(self.ArdIMU.getEastPID()) < threshold)
        self.DownLocked = (abs(self.ArdIMU.getDownPID()) < threshold)
        if self.NorthLocked and self.EastLocked and self.NorthLocked:
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within position threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.PositionRunning = False
        else:
            self.InitialTime = time.perf_counter()
        return self.PositionRunning

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

    def descend_A_Bit(self, safety=True):
        startdistance = self.Sonar.getStartingDistance()
        self.Sonar.update()
        distance = self.Sonar.getDistance()
        starttime = time.perf_counter()
        while (distance - (startdistance - startdistance / 5)) < 30:
            if safety:
                if time.perf_counter() > starttime + 20:
                    break
            self.Sonar.update()
            self.setThrusters_GyroSonarPID()
        pass

    def BasicDirectionPower(self, index, power=30):
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
        if self.UsingGyro:
            # self.UpdateGyro()
            self.ArdIMU.UpdateAngle()
            self.setThrusters_GyroPID()
            self.GyroLocking()
        else:
            # self.UpdateThrusters()
            pass

    def updateSensors(self):
        if self.UsingGyro:
            self.ArdIMU.update()
            if self.UsingSonar:
                self.Sonar.update()
                self.setThrusters_GyroSonarPID()
            else:
                self.setThrusters_GyroPID()
            pass

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
            self.ArdIMU.CalculatePID()

    def UpdateThrusters(self):
        if self.UsingGyro:
            # self.ArdIMU.update()
            if self.UsingSonar:
                # self.Sonar.update()
                self.setThrusters_GyroSonarPID()
            else:
                self.setThrusters_GyroPID()
        self.SendThrusterCommands()
        # time.sleep(.1)

    def setThrusters_GyroPID(self):
        # self.ArdIMU.CalculateError(self.YawOffset, self.PitchOffset, self.RollOffset,
        #                            self.NorthOffset, self.EastOffset, self.DownOffset)
        self.Thruster_VentralLB.setSpeedPID(-self.VentralPowerLB,
                                            # + self.ArdIMU.getPitchPID()
                                            # + self.ArdIMU.getRollPID()
                                            )
        self.Thruster_VentralRB.setSpeedPID(- self.VentralPowerRB,
                                            # + self.ArdIMU.getPitchPID()
                                            # - self.ArdIMU.getRollPID()
                                            )
        self.Thruster_VentralLF.setSpeedPID(-self.VentralPowerLF,
                                            # - self.ArdIMU.getPitchPID()
                                            # + self.ArdIMU.getRollPID()
                                            )
        self.Thruster_VentralRF.setSpeedPID(- self.VentralPowerRF,
                                            # - self.ArdIMU.getPitchPID()
                                            # - self.ArdIMU.getRollPID()
                                            )

        self.Thruster_LateralBL.setSpeedPID(-self.LateralPowerBL,
                                            self.ArdIMU.getYawPID()
                                            )
        self.Thruster_LateralFL.setSpeedPID(0,
                                            0)
        self.Thruster_LateralBR.setSpeedPID(self.LateralPowerBR,
                                            self.ArdIMU.getYawPID()
                                            )
        self.Thruster_LateralFR.setSpeedPID(0,
                                            0)

        # saving old thrust code
        # self.Thruster_LateralBL.setSpeedPID(self.LateralPowerBL,
        #                                     self.ArdIMU.getYawPID())
        # self.Thruster_LateralFL.setSpeedPID(self.LateralPowerFL,
        #                                     self.ArdIMU.getYawPID())
        # self.Thruster_LateralBR.setSpeedPID(self.LateralPowerBR,
        #                                     -self.ArdIMU.getYawPID())
        # self.Thruster_LateralFR.setSpeedPID(self.LateralPowerFR,
        #                                     -self.ArdIMU.getYawPID())
        self.SendThrusterCommands()
        # time.sleep(.1)

    def DriveThroughGate(self):
        self.passedGate = False
        while not self.passedGate:
            self.Vision.process_image(1)

    def UpdateThrusters_Gyro_PID_NoCompass(self):
        # self.ArdIMU.CalculateError(self.YawOffset, self.PitchOffset, self.RollOffset,
        #                            self.NorthOffset, self.EastOffset, self.DownOffset)
        self.Thruster_VentralLB.setSpeedPID(-self.VentralPowerLB,
                                            + self.ArdIMU.getPitchPID()
                                            + self.ArdIMU.getRollPID())
        self.Thruster_VentralRB.setSpeedPID(- self.VentralPowerRB,
                                            + self.ArdIMU.getPitchPID()
                                            - self.ArdIMU.getRollPID())
        self.Thruster_VentralLF.setSpeedPID(-self.VentralPowerLF,
                                            - self.ArdIMU.getPitchPID()
                                            + self.ArdIMU.getRollPID())
        self.Thruster_VentralRF.setSpeedPID(- self.VentralPowerRF,
                                            - self.ArdIMU.getPitchPID()
                                            - self.ArdIMU.getRollPID())

        self.Thruster_LateralBL.setSpeedPID(self.LateralPowerBL, 0)
        self.Thruster_LateralFL.setSpeedPID(self.LateralPowerFL, 0)
        self.Thruster_LateralBR.setSpeedPID(self.LateralPowerBR, 0)
        self.Thruster_LateralFR.setSpeedPID(self.LateralPowerFR, 0)
        self.SendThrusterCommands()
        # time.sleep(.1)

    def setThrusters_GyroSonarPID(self):
        # self.ArdIMU.CalculateError(self.YawOffset, self.PitchOffset, self.RollOffset,
        #                            self.NorthOffset, self.EastOffset, self.DownOffset)

        self.Thruster_VentralLB.setSpeedPID(-self.VentralPowerLB,
                                            + self.ArdIMU.getPitchPID()
                                            + self.ArdIMU.getRollPID()
                                            - self.Sonar.getPID())
        self.Thruster_VentralRB.setSpeedPID(- self.VentralPowerRB,
                                            + self.ArdIMU.getPitchPID()
                                            - self.ArdIMU.getRollPID()
                                            - self.Sonar.getPID())
        self.Thruster_VentralLF.setSpeedPID(-self.VentralPowerLF,
                                            - self.ArdIMU.getPitchPID()
                                            + self.ArdIMU.getRollPID()
                                            - self.Sonar.getPID())
        self.Thruster_VentralRF.setSpeedPID(- self.VentralPowerRF,
                                            - self.ArdIMU.getPitchPID()
                                            - self.ArdIMU.getRollPID()
                                            - self.Sonar.getPID())

        self.Thruster_LateralBL.setSpeedPID(self.LateralPowerBL, 0)
        self.Thruster_LateralFL.setSpeedPID(self.LateralPowerFL, 0)
        self.Thruster_LateralBR.setSpeedPID(self.LateralPowerBR, 0)
        self.Thruster_LateralFR.setSpeedPID(self.LateralPowerFR, 0)
        self.SendThrusterCommands()
        # time.sleep(.1)

    def UpdateThrustersVisionPID(self):

        self.Thruster_LateralBL.setSpeedPID(self.LateralPowerBL, pid=self.Vision.getXPID())
        self.Thruster_LateralFL.setSpeedPID(self.LateralPowerFL, pid=self.Vision.getXPID())
        self.Thruster_LateralBR.setSpeedPID(self.LateralPowerBR, pid=-self.Vision.getXPID())
        self.Thruster_LateralFR.setSpeedPID(self.LateralPowerFR, pid=-self.Vision.getXPID())

        self.Thruster_VentralLB.setSpeedPID(self.VentralPowerLB,
                                            pid=self.Vision.getYPID())
        self.Thruster_VentralRB.setSpeedPID(self.VentralPowerRB,
                                            pid=self.Vision.getYPID())
        self.Thruster_VentralLF.setSpeedPID(self.VentralPowerLF,
                                            pid=-self.Vision.getYPID())
        self.Thruster_VentralRF.setSpeedPID(self.VentralPowerRF,
                                            pid=-self.Vision.getYPID())
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
                                                      self.Thruster_VentralRF.getSpeed() + self.VentralPowerRF_offset
                                                      )

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
        # print("Writing out file...")
        # self.writeout.close()
        # time.sleep(1)
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
    def setSpeedPID(self, speed, pid=0.0):
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
