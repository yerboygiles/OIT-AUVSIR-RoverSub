#!python3
# Author: Theodor Giles
# Created: 8/7/20
# Last Edited 7/23/22
# Description:
# manages sonar data, updates, calls, passing to other nodes, etc.

from brping import Ping1D
import time


class Sonar:
    StringIn = ""
    Distance = 0.0
    Confidence = 0.0
    Error = 0.0
    Previous_Error = 0.0
    Error_Sum = 0.0
    Error_Delta = 0.0
    PingDistance = 0.0

    Kp = .05  # constant to modify PID
    Ki = .1  # constant to modify PID
    Kd = .3  # constant to modify PID

    P = 0.0
    I = 0.0
    D = 0.0
    PID = 0.0

    def __init__(self, serial="/dev/ttyUSB0", id=0, name=""):
        # read info from vehicle
        self.ID = id
        self.name = ""

        self.Ping = Ping1D()
        self.Ping.connect_serial(serial, 115200)
        # self.StartingDistance = self.getDistance()
        self.DistanceOffset = 0
        distances = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        i = 0
        print("Calibrating sonar initial...")
        while i < 10:
            confidence = self.update()
            distance = self.getDistance()
            if confidence < 65 and distance != 0.0:
                print("Confidence low...")
                time.sleep(0.5)
                i = i - 1
            else:
                distances[i] = self.getDistance()
                print("calib distance: ", distances[i])
            i = i + 1
        sum = 0
        for i in range(10):
            sum = sum + distances[i]
        self.DistanceOffset = sum / 10
        # print('Starting Distance: ', self.StartingDistance)
        self.Ping.set_speed_of_sound(1450000)
        print('Starting sonar, wait 5...')
        time.sleep(5)

        # - Read the actual depth:
        # time.sleep(3)

    # gets hardware info
    def getInfo(self):
        return self.name

    # position read when starting the RoboSub
    def getStartingDistance(self):
        return self.StartingDistance

    # current gyro read
    def getDistance(self):
        return self.Distance

    def update(self):
        data = self.Ping.get_distance()
        distance = data["distance"]
        confidence = data["confidence"]
        if confidence > 65:
            self.Distance = distance
        self.CalculateError()
        self.CalculatePID()
        return confidence

    # req for PID calculation
    def CalculateError(self):
        # previous error for error delta
        # gyro
        self.Previous_Error = self.Error

        # error for proportional control
        self.Error = self.Distance - self.DistanceOffset

        # sum of error for integral
        self.Error_Sum = self.Error_Sum + self.Error

        # math for change in error to do derivative
        self.Error_Delta = self.Error - self.Previous_Error

    def setOffset(self, offset):
        self.DistanceOffset = offset

    def setOffsetCurrent(self):
        self.DistanceOffset = self.getDistance()

    # pid calculation
    def CalculatePID(self):
        # Yaw PID variable setting
        self.P = (self.Error * self.Kp)
        self.I = 0
        # self.I = (self.Error_Sum * self.Ki)
        self.D = 0
        # self.D = (self.Error_Delta * self.Kd)
        self.PID = self.P + self.I + self.D

    def getPID(self):
        return self.PID

    def getP(self):
        return self.P

    def getI(self):
        return self.I

    def getD(self):
        return self.D

    # def Terminate(self):
    #     self.Ping.
