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
    def __init__(self, filename, usingarduino, usingvision, usinggyro, usingsim):
        self.Input = False
        if filename == 0:
            self.input = filename
        else:
            self.Filename = filename
        self.UsingArduino = usingarduino
        self.UsingVision = usingvision
        self.UsingGyro = usinggyro
        self.UsingSim = usingsim
        self.Active = False
        self.Movement = NavigationCommander(self.UsingArduino,
                                            self.UsingVision,
                                            self.UsingGyro,
                                            self.UsingSim,
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
        for i in range(100):
            perftime = time.perf_counter()
            self.Movement.GyroTesting()
            self.Movement.ArduinoTesting()
            print("Loop time: ", time.perf_counter() - perftime)
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
