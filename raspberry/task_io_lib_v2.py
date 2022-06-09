#!python3
# Author: Theodor Giles
# Created: 8/7/20
# Last Edited 5/18/22
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
        self.Movement.ZeroSonar()
        starttime = time.perf_counter()
        with open('telemetry.txt', 'a') as f:
            while (time.perf_counter() - starttime) < 60:
                perftime = time.perf_counter()
                confidence = self.Movement.Sonar.updateDistance()
                distance = self.Movement.Sonar.getDistance()
                self.Movement.GyroTesting()
                self.Movement.ArduinoTesting()

                towrite = "Sonar dist, confidence" + str(distance) + str(confidence)
                f.write(towrite)
                towrite = "Sonar PID: " + str(self.Movement.Sonar.getPID())
                f.write(towrite)
                towrite = "IMU angles: " + str(self.Movement.ArdIMU.getAngle())
                f.write(towrite)
                towrite = "IMU Pitch, Roll PID: " + str(self.Movement.ArdIMU.getPitchPID()) + \
                          str(self.Movement.ArdIMU.getRollPID())
                f.write(towrite)
                f.write("Runtime: ")
                f.write(str(time.perf_counter() - starttime))
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
