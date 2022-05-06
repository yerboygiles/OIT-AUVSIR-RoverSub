#!python3
# Author: Theodor Giles
# Created: 4/10/21
# Last Edited 5/4/22
# Description:
# This node manages the commands/movement/physical
# control of the RoboSub V2, 2020-21

import serial


class ArduinoCommander:

    def __init__(self, serialport='/dev/ttyAMA0'):
        self.serial = serial.Serial('/dev/serial0', 115200)

    # new protocol for ard. communication -
    # by having a single charac at the beginning of string, we can improve the parse speed of the arduino. instead of
    # searching the huge list of hardpoints, we can classify each and only search in the places we want.
    # comm codes for hardpoints:
    # t - thruster
    # g - gyro
    # s - servo
    #
    # bring in thruster vals as 3-char strings

    def CommunicateAllThrusters(self, lLBspeed, lRBspeed, lLFspeed, lRFspeed, vLBspeed, vLFspeed, vRBspeed, vRFspeed):
        confirm = False
        # send out data
        outdata = ""
        outdata += 't'
        outdata += 'a'
        outdata += ','
        outdata += str(lLBspeed)
        outdata += ','
        outdata += str(lRBspeed)
        outdata += ','
        outdata += str(lLFspeed)
        outdata += ','
        outdata += str(lRFspeed)

        outdata += ','
        outdata += str(vLBspeed)
        outdata += ','
        outdata += str(vRBspeed)
        outdata += ','
        outdata += str(vLFspeed)
        outdata += ','
        outdata += str(vRFspeed)

        outdata += "\n"

        self.serial.write(outdata.encode('ascii'))
        print("Outdata: ", outdata.encode('ascii'))
        # confirm data received
        self.serial.reset_input_buffer()
        while self.serial.inWaiting() > 0:
            response = self.serial.readline()
            if response == "ATD":
                print("Thruster values sent, handshake confirmed.")
                confirm = False
            else:
                print("Thruster values sent, handshake unconfirmed.")
                confirm = False
        return confirm

    def Communicate_Thrusters(self, thrustid, speed):
        switch = {
            'lLB': self.Communicate_lLB(speed),
            'lRB': self.Communicate_lRB(speed),
            'lLF': self.Communicate_lLF(speed),
            'lRF': self.Communicate_lRF(speed),

            'vLB': self.Communicate_lLB(speed),
            'vRB': self.Communicate_lRB(speed),
            'vLF': self.Communicate_lLF(speed),
            'vRF': self.Communicate_lRF(speed)
        }
        switch.get(thrustid)

    def Communicate_lLB(self, speed):
        sendstring = ""
        sendstring += 't'  # thruster
        sendstring += 'l'  # orientation
        sendstring += "LB"  # thruster id
        sendstring += speed
        self.serial.write(sendstring.encode('utf-8'))

    def Communicate_lRB(self, speed):
        sendstring = ""
        sendstring += 't'  # thruster
        sendstring += 'l'  # orientation
        sendstring += "RB"  # thruster id
        sendstring += speed
        self.serial.write(sendstring.encode('utf-8'))

    def Communicate_lLF(self, speed):
        sendstring = ""
        sendstring += 't'  # thruster
        sendstring += 'l'  # orientation
        sendstring += "LF"  # thruster id
        sendstring += speed
        self.serial.write(sendstring.encode('utf-8'))

    def Communicate_lRF(self, speed):
        sendstring = ""
        sendstring += 't'  # thruster
        sendstring += 'l'  # orientation
        sendstring += "RF"  # thruster id
        sendstring += speed
        self.serial.write(sendstring.encode('utf-8'))

    def Communicate_vLB(self, speed):
        sendstring = ""
        sendstring += 't'  # thruster
        sendstring += 'l'  # orientation
        sendstring += "LB"  # thruster id
        sendstring += speed
        self.serial.write(sendstring.encode('utf-8'))

    def Communicate_vRB(self, speed):
        sendstring = ""
        sendstring += 't'  # thruster
        sendstring += 'l'  # orientation
        sendstring += "RB"  # thruster id
        sendstring += speed
        self.serial.write(sendstring.encode('utf-8'))

    def Communicate_vLF(self, speed):
        sendstring = ""
        sendstring += 't'  # thruster
        sendstring += 'l'  # orientation
        sendstring += "LF"  # thruster id
        sendstring += speed
        self.serial.write(sendstring.encode('utf-8'))

    def Communicate_vRF(self, speed):
        sendstring = ""
        sendstring += 't'  # thruster
        sendstring += 'v'  # orientation
        sendstring += "RF"  # thruster id
        sendstring += speed
        self.serial.write(sendstring.encode('utf-8'))

    def SendToArduino(self, whattosend):
        self.serial.write(whattosend.encode('utf-8'))
