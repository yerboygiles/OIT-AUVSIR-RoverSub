#!python3
# Author: Theodor Giles
# Created: 4/10/21
# Last Edited 5/19/22
# Description:
# This node manages the communication with the arduino

import serial


class ArduinoCommander:

    def __init__(self):
        self.serial = serial.Serial('/dev/ttyS0', 115200, timeout=0.3)
        try:
            self.serial.open()
        except serial.serialutil.SerialException:
            print("Port already open.")
            pass

    # new protocol for ard. communication -
    # by having a single charac at the beginning of string, we can improve the parse speed of the arduino. instead of
    # searching the huge list of hardpoints, we can classify each and only search in the places we want.
    # comm codes for hardpoints:
    # t - thruster
    # g - gyro
    # s - servo
    #
    # bring in thruster vals as 3-char strings
    def ReadFromArd(self):
        response = ""
        response = self.serial.readline()
        if response != "":
            print("Response: ", response)

    def getSerial(self):
        return self.serial

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

        # outdata += "}"
        outdata += "\n"
        while 1:
            try:
                self.serial.write(outdata.encode('ascii'))
                break
            except:
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
        return confirm

    def SendToArduino(self, whattosend):
        whattosend += "\n"
        self.serial.write(whattosend.encode('utf-8'))

        # print("Rear angle:", data)
