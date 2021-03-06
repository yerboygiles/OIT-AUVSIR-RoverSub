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
    # self, filename, usingarduino, usingvision, usinggyro, usingsim
    def __init__(self, filename, usingarduino, usingvision, usinggyro, usingsim, usingping):
        self.Input = False
        if filename == 0:
            self.input = filename
        else:
            self.Filename = filename
        self.UsingArduino = usingarduino
        self.UsingVision = usingvision
        self.UsingGyro = usinggyro
        self.UsingSim = usingsim
        self.UsingPing = usingping
        self.Active = False
        self.Movement = NavigationCommander(self.UsingArduino,
                                            self.UsingVision,
                                            self.UsingGyro,
                                            self.UsingSim,
                                            self.UsingPing,
                                            False)
        self.CommandList = []

    # get tasks from the .txt and completes them
    def get_tasks(self, input=False):
        # Testing
        if not input:
            self.Commands = open(self.Filename)
            for CommandLine in self.Commands:
                print("CommandLine: ", CommandLine)
                self.CommandList.append(CommandLine)
            print("Commands read...")
            self.Commands.close()
            print("Commands: ", self.CommandList)
            self.Movement.receiveCommands(self.CommandList)
            self.active = False
        else:
            if self.Input != -1:
                pass
            else:
                print("Waiting for input...")

            self.CommandList.append(self.Input)

    def testData(self):
        loopi = 0
        # self.Movement.ZeroSonar()
        starttime = time.perf_counter()
        with open('telemetry_active.txt', 'w') as f:
            self.Movement.Vision.startStream()
            while (time.perf_counter() - starttime) < 30:
                print("Loop num: ", loopi)
                loopi = loopi + 1
                print(time.perf_counter() - starttime)
                perftime = time.perf_counter()
                # confidence = self.Movement.Sonar.updateDistance()
                # distance = self.Movement.Sonar.getDistance()
                # self.Movement.GyroTesting()
                self.Movement.VisionTesting()
                print("Vision ran...")
                self.Movement.GyroTesting()
                print("Gyro ran...")
                # self.Movement.ArduinoTesting()
                if self.Movement.Vision.SeenTarget:
                    towrite = "Seen target: " + self.Movement.Vision.SeenTarget
                    print(towrite)
                    f.write(towrite)
                    f.write("\n")
                    towrite = "X, Y vision offset: " + self.Movement.Vision.getXOffset() + ", " + self.Movement.Vision.getYOffset()
                    print(towrite)
                else:
                    towrite = "Target not visible."
                    print(towrite)
                f.write(towrite)
                f.write("\n")

                # towrite = "Sonar dist, confidence" + str(distance) + str(confidence)
                # f.write(towrite)
                # f.write("\n")
                # towrite = "Sonar PID: " + str(self.Movement.Sonar.getPID())
                # f.write(towrite)
                # f.write("\n")

                towrite = "IMU Angles: " + str(self.Movement.ArdIMU.getAngle())
                f.write(towrite)
                f.write("\n")

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
