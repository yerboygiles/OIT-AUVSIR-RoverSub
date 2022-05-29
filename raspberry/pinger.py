from brping import Ping1D


class Pinger:
    StringIn = ""
    Distance = 0.0
    Confidence = 0.0
    Error = 0.0
    Previous_Error = 0.0
    Error_Sum = 0.0
    Error_Delta = 0.0
    PingDistance = 0.0

    Kp = .3  # constant to modify PID
    Ki = .2  # constant to modify PID
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
        self.StartingDistance = self.getDistance()
        self.DistanceOffset = self.StartingDistance
        print('Starting Distance: ', self.StartingDistance)

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
        data = self.Ping.get_distance()
        return data["distance"]

    # req for PID calculation
    def CalculateError(self):
        # previous error for error delta
        # gyro
        self.Previous_Error = self.Error

        # error for proportional control
        self.Error = self.PingDistance - self.DistanceOffset

        # sum of error for integral
        self.Error_Sum = self.Error_Sum + self.Error

        # math for change in error to do derivative
        self.Error_Delta = self.Error - self.Previous_Error

    def setOffset(self, offset):
        self.DistanceOffset = offset

    def setOffsetCurrent(self):
        self.DistanceOffset = self.getDistance()

    # pid calculation
    def CalcPID(self):
        # Yaw PID variable setting
        self.P = (self.Error * self.Kp)
        self.I = (self.Error_Sum * self.Ki)
        self.D = (self.Error_Delta * self.Kd)
        self.PID = self.P + self.I + self.D

    def UpdateDistance(self):
        distance = self.getDistance()
        self.CalculateError()
        self.CalcPID()

    def getPID(self):
        return self.PID

    def getP(self):
        return self.P

    def getI(self):
        return self.I

    def getD(self):
        return self.D
