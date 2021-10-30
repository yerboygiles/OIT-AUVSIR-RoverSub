#! /usr/env/python

'''
# 
# Phidget Hello World program for all devices
# (c) Phidgets 2012
#
'''

from ctypes import *
import sys

from Phidget22.PhidgetException import *
from Phidget22.Devices import *
from Phidget22.Devices.Manager import *
from Phidget22.Phidget import *

# ========== Event Handling Functions ==========

def AttachHandler(self, channel):

    attachedDevice = channel
    serialNumber = attachedDevice.getDeviceSerialNumber()
    deviceName = attachedDevice.getDeviceName()
    print("Hello to Device " + str(deviceName) + ", Serial Number: " + str(serialNumber))

def DetachHandler(self, channel):
    detachedDevice = channel
    serialNumber = detachedDevice.getDeviceSerialNumber()
    deviceName = detachedDevice.getDeviceName()
    print("Goodbye Device " + str(deviceName) + ", Serial Number: " + str(serialNumber))
 
# =========== Python-specific Exception Handler ==========        
        
def LocalErrorCatcher(e):
    print("Phidget Exception: " + str(e.code) + " - " + str(e.details) + ", Exiting...")
    exit(1)

# ========= Main Code ==========        
        
try: manager = Manager()
except RuntimeError as e:
    print("Runtime Error " + e.details + ", Exiting...\n")
    exit(1)

try:
    #logging example, uncomment to generate a log file
    #manager.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")
    manager.setOnAttachHandler(AttachHandler)
    manager.setOnDetachHandler(DetachHandler)
except PhidgetException as e: LocalErrorCatcher(e)

print("Opening....")
try:
    manager.open()
except PhidgetException as e: LocalErrorCatcher(e)

print("Phidget Simple Playground (plug and unplug devices)");
print("Press Enter to end anytime...");
character = sys.stdin.read(1)

print("Closing...")
try:
    manager.close()
except PhidgetException as e: LocalErrorCatcher(e)

exit(0)

