import sys
import time 
from Phidget22.Devices.Dictionary import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *

try:
    dict = Dictionary()
except RuntimeError as e:
    print("Runtime Exception %s" % e.details)
    print("Press Enter to Exit...\n")
    readin = sys.stdin.read(1)
    exit(1)

def DictionaryAdd(self, key, value):
    try:
        print("\n--------------")
        print("Key Added:")
        print("Key: " + key)
        print("Value: " + value)

    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)   
    
def DictionaryUpdate(self, key, value):
    try:
        print("\n--------------")
        print("Key Updated:")
        print("Key: " + key)
        print("Value: " + value)

    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)

def DictionaryRemove(self, key):
    try:
        print("\n--------------")
        print("Key Removed:")
        print("Key: " + key)

    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)

#declare the main example first, so we can declare other functions after
def main():
    try:
        dict.setOnAddHandler(DictionaryAdd)
        dict.setOnUpdateHandler(DictionaryUpdate)
        dict.setOnRemoveHandler(DictionaryRemove)

        print("This example requires that you have already created a dictionary labeled")
        print("\"testdict\" in the control panel for the server hosting your dicitonary.")

        Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)

        dict.setDeviceLabel("testdict")

        print("\nWaiting for the Phidget Dicitonary Object to be attached...")
        dict.openWaitForAttachment(20000)

        #Set can be used to set the a value for a key even if the key doesn't exist yet
        dict.set("key1", "value")
        result = dict.get("key1")
        print("Result 1: " + result)

        time.sleep(3)

        dict.set("key1", "set 2")
        result = dict.get("key1")
        print("Result 2: " + result)

        time.sleep(3)

        #Update can be used to update the value of pre-existing keys
        dict.update("key1", "updated")
        result = dict.get("key1")
        print("Result 3: " + result)

        time.sleep(3)

        #As you might expect, Remove removes keys from the dictionary
        print("Removing key1")
        dict.remove("key1")

        time.sleep(3)

        #You can add keys explicitly, to only add them if they don't yet exist.
        dict.add("key1", "added key1")
        result = dict.get("key1")
        print("Result 4: " + result)

        time.sleep(3)

        print("Removing key1")
        dict.remove("key1")

        print("\nListing Keys: \n")

        listKeys(dict)

        print("\nActive demonstration complete.\n")
        print("Now would be a good time to try out the events by running a second copy of this")
        print("program, or using the control pannel example to edit key-value pairs in the")
        print("test dictionary.\n")

        print("")

        print("\nPress Enter to end anytime...\n")
        readin = sys.stdin.read(1)

        dict.close()
        print("\nClosed Dictionary")

    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)

    try:
        dict.close()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1) 
    print("Closed CurrentInput device")
    exit(0)
             
def listKeys(dict):
    try: 
        start = ""
        length = 0

        newSources = dict.scan(start)
        while len(newSources) > 0:
            sourceList = newSources.split("\n")
            sourceList = [x for x in sourceList if x] #remove any empty strings
            for i in range (0, len(sourceList)):
                print(sourceList[i])
            start = sourceList[len(sourceList)- 1]
            newSources = dict.scan(start)

    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1) 

main() #Run the main example     