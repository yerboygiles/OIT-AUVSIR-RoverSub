#!python3
# Author: Theodor Giles
# Created: 8/7/20
# Last Edited 7/23/22
# Description:
# This program manages the conversion of the mission.txt into commands that the MovementCommander can understand
# as well as the AI/TF/vision integration
# Allows the movement_commander to update at all times, causing no lag for switching commands
import time
from vector_commander import NavigationCommander


class TaskIO:
    # filename can first be sent a 0 to initialize string-input commands
    # instead of
    # self, filename, usingvision, usinggyro, usingsim
    def __init__(self, filename, usingvision, usinggyro, usingsim, usingsonar):
        self.auto_state = 0
        self.Input = False
        if filename == 0:
            self.input = filename
        else:
            self.Filename = filename
        self.UsingVision = usingvision
        self.UsingGyro = usinggyro
        self.UsingSim = usingsim
        self.UsingSonar = usingsonar
        self.Active = False
        self.Movement = NavigationCommander(self.UsingVision,
                                            self.UsingGyro,
                                            self.UsingSim,
                                            self.UsingSonar,
                                            False)
        self.CommandList = []

    # get tasks from the .txt and completes them
    # def get_tasks(self, input=False):
    #     # Testing
    #     if not input:
    #         self.Commands = open(self.Filename)
    #         for CommandLine in self.Commands:
    #             print("CommandLine: ", CommandLine)
    #             self.CommandList.append(CommandLine)
    #         print("Commands read...")
    #         self.Commands.close()
    #         print("Commands: ", self.CommandList)
    #         self.Movement.receiveCommands(self.CommandList)
    #         self.active = False
    #     else:
    #         if self.Input != -1:
    #             pass
    #         else:
    #             print("Waiting for input...")
    #
    #         self.CommandList.append(self.Input)
    def autonomousStMch(self):
        autonomous = True
        setpid = True
        starttime = time.perf_counter()
        while autonomous:
            self.Movement.updateSensors()
            self.Movement.setThrusters_GyroSonarPID()
            if self.auto_state == 0:
                if setpid:
                    # descend to 3/5 height of pool
                    self.Movement.setSonarOffset(5, 3)
                    self.Movement.Sonar.setOffsetCurrent((self.Movement.Sonar.getStartingDistance() / 5) * 3)
                    setpid = False
                if not self.Movement.SonarLocking(self.Movement.Sonar.getStartingDistance() / 300, 5):
                    setpid = True
                    # self.auto_state = 1
                    autonomous = False
                self.auto_state = 1
            # elif self.auto_state == 1:
            # if setpid:
            #     # forwards
            #     self.Movement.BasicDirectionPower(1)
            # if (time.perf_counter() - starttime) < 30:
            #     autonomous = False

    def autonomousStMch_alt(self):
        try:
            autonomous = True
            setpid = True
            starttime = time.perf_counter()
            self.Movement.InitialTime = starttime
            while autonomous:
                self.Movement.updateSensors()
                print("Yaw: ", self.Movement.ArdIMU.Angle[0])
                print("FL, FR", self.Movement.Thruster_LateralBL.getSpeed(), " : ", self.Movement.Thruster_LateralBR.getSpeed())

                if self.auto_state == 0:
                    self.Movement.BasicDirectionPower(1)
                    if (time.perf_counter() - starttime) > 30:
                        autonomous = False
                    self.auto_state = 1
                self.Movement.UpdateThrusters()
        except KeyboardInterrupt:
            self.Movement.Terminate()

    def testData(self):
        loopi = 0
        if self.UsingSonar:
            self.Movement.ZeroSonar()
        starttime = time.perf_counter()
        with open('telemetry_active.txt', 'w') as f:
            if self.UsingVision:
                self.Movement.Vision.startStream()
            while (time.perf_counter() - starttime) < 30:
                # print("Loop num: ", loopi)
                loopi = loopi + 1
                # print(time.perf_counter() - starttime)
                # perftime = time.perf_counter()
                if self.UsingSonar:
                    confidence = self.Movement.Sonar.update()
                    distance = self.Movement.Sonar.getDistance()
                # self.Movement.VisionTesting()
                # print("Vision ran...")
                if self.UsingGyro:
                    self.Movement.ArdIMU.update()
                # print("Gyro ran...")
                # if self.UsingVision:
                #     self.Movement.UpdateThrustersGyroVisionPID()
                if self.UsingGyro:
                    if self.UsingSonar:
                        # self.Movement.UpdateThrusters_GyroSonar_PID()
                        pass
                    else:
                        # self.Movement.UpdateThrusters_Gyro_PID()
                        pass
                    pass
                # elif self.UsingGyro:
                #     self.Movement.UpdateThrusters_Gyro_PID()
                # self.Movement.ArduinoTesting()
                if self.UsingVision:
                    if self.Movement.Vision.SeenTarget:
                        towrite = "Visible Target: " + self.Movement.Vision.SeenTarget
                        print(towrite)
                        f.write(towrite)
                        f.write("\n")
                        towrite = "X, Y vision offset: " + self.Movement.Vision.getXOffset() + ", " + \
                                  self.Movement.Vision.getYOffset()
                        print(towrite)
                    else:
                        towrite = "Target not visible."
                        print(towrite)
                    f.write(towrite)
                    f.write("\n")
                if self.UsingSonar:
                    towrite = "Sonar dist, confidence" + str(distance) + str(confidence)
                    f.write(towrite)
                    f.write("\n")
                    print(towrite)
                    towrite = "Sonar PID: " + str(self.Movement.Sonar.getPID())
                    f.write(towrite)
                    f.write("\n")
                    print(towrite)
                    # swaws
                if self.UsingGyro:
                    towrite = "IMU Angles: " + str(self.Movement.ArdIMU.getAngle())
                    f.write(towrite)
                    f.write("\n")
                    print(towrite)
                    towrite = "IMU PIDs: " + str(self.Movement.ArdIMU.getYawPID()) + \
                              ", " + str(self.Movement.ArdIMU.getPitchPID()) + \
                              ", " + str(self.Movement.ArdIMU.getRollPID())
                    f.write(towrite)
                    f.write("\n")
                    print(towrite)

                # towrite = "IMU Acceleration: " + str(self.Movement.ArdIMU.getAcceleration())
                # f.write(towrite)
                # towrite = "IMU Position: " + str(self.Movement.ArdIMU.getPosition())
                # f.write(towrite)
                # f.write("\n")
                # towrite = "IMU North, East, Down PID: " + str(self.Movement.ArdIMU.getNorthPID()) + "," + \
                #           str(self.Movement.ArdIMU.getEastPID()) + "," + \
                #           str(self.Movement.ArdIMU.getDownPID())
                # # print(towrite)
                # f.write(towrite)

                f.write("\n")
                f.write("Runtime: ")
                f.write(str(time.perf_counter() - starttime))
                f.write("\n")
            f.close()
        # perftime = time.perf_counter()
        # self.Movement.BasicDriverControl()
        # self.Movement.ResetGyro()
        # for i in range(3000):
        #     self.Movement.GyroTesting()

    def testArduino(self):
        self.Movement.ArduinoTesting()

    # end processes and mission
    def terminate(self):
        self.Active = False
        self.Movement.Terminate()
