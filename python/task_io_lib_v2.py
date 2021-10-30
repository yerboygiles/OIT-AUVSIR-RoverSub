#!python3
# Author: Theodor Giles
# Created: 8/7/20
# Last Edited 11/22/20
# Description:
# This program manages the conversion of the mission.txt into commands that the MovementCommander can understand
# as well as the AI/TF/vision integration
# Allows the movement_commander to update at all times, causing no lag for switching commands
import time
from vector_commander import NavigationCommander


class TaskIO:
    # init
    def __init__(self, filename, usingvision, usinggyro, usingsim):
        self.Filename = filename
        self.UsingVision = usingvision
        self.UsingGyro = usinggyro
        self.UsingSim = usingsim
        self.Active = False
        self.Movement = NavigationCommander(self.UsingVision, self.UsingGyro, self.UsingSim)
        self.CommandList = []

    # get tasks from the .txt and completes them
    def get_tasks(self):
        # Testing
        self.Commands = open(self.Filename)
        for CommandLine in self.Commands:
            self.CommandList.append(CommandLine)
        print("Commands read...")
        self.Commands.close()
        print("Commands: ", self.CommandList)
        self.Movement.receiveCommands(self.CommandList)
        self.active = False

    # end processes and mission
    def terminate(self):
        self.Active = False
        self.Movement.Terminate()
