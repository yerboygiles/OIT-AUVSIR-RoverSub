import sys
import time
from Phidget22.Devices.Manager import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Devices.Log import *
from Phidget22.LogLevel import *

#declare the main example first, so we can declare other demo functions after
def main():
    try:
        print("\nLogging to File\n")

        #First, we enable the log.
        #Here, we'll log to a file called log.txt in the same folder as the program
        Log.enable(LogLevel.PHIDGET_LOG_INFO, "log.txt")
    
        #You can send your own messages to the Phidget log
        Log.log(LogLevel.PHIDGET_LOG_ERROR, "Custom Log Error")

        Log.log(LogLevel.PHIDGET_LOG_INFO, "Custom Log Info")
    
        print("\nNow you can open the log file to view the results\n")

        Log.disable()

        time.sleep(10)

    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1) 
    exit(0)

# Demo function implementation. Here is source code to demonstrate the more obscure
# functionality of the Log object. You can call these functions from main() to try them out 

def demo_log_level():
    try:
        #Log levels serve to filter out unwanted information from the log
        #To demonstrate, we'll enable the log with log level LogLevel.PHIDGET_LOG_INFO
        Log.enable(LogLevel.PHIDGET_LOG_INFO, "log_level_log.txt")    

        print("\nDemonstrating log level LogLevel.PHIDGET_LOG_INFO\n")    
        print("You should see both log messages in log_level_log.txt\n")    

        #You can send your own messages to the Phidget log
        Log.log(LogLevel.PHIDGET_LOG_INFO, "Custom Log Info 0")    

        Log.log(LogLevel.PHIDGET_LOG_ERROR, "Custom Log Error 1")    

        #Now we'll repeat the precess, using log level ERROR
        Log.setLevel(LogLevel.PHIDGET_LOG_ERROR)    

        print("\nDemonstrating log level LogLevel.PHIDGET_LOG_ERROR\n")    
        print("You should only see the log message produced by LogLevel.PHIDGET_LOG_ERROR.\n")    

        #Note that the message flagged as INFO will not appear when using log level ERROR
        #This helps filter out unwanted informaiton from the log
        Log.log(LogLevel.PHIDGET_LOG_INFO, "Custom Log Info 2")    

        Log.log(LogLevel.PHIDGET_LOG_ERROR, "Custom Log Error 3")    

        Log.disable()    

        time.sleep(5)

    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1) 

def demo_rotation():
    try:
        #We'll set up a rotation to keep the file size to 50kB max, 
        #and allow storage of the latest 3 log files
        Log.setRotating(50000, 3)    

        #Here, we'll log to a file called rotation_log.txt in the same folder as the C program
        Log.enable(LogLevel.PHIDGET_LOG_INFO, "rotation_log.txt")    

        print("\nLogging to rotation_log.txt File...\n")    
        #Now we'll log 1000 messages to the log file
        #Once complete, open the file to see the results of logging and rotation
        for i in range(0, 1000): 
            Log.log(LogLevel.PHIDGET_LOG_INFO, "Custom Log Sequential Logs: " + str(i))
            time.sleep(0.002)

        Log.disable()    

        print("\nNow you can open rotation_log.txt to view the results\n")    

        time.sleep(5)
         
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)    

def demo_source_level():
    try:
        #Setting up a manager to be used to demonstrate logging
        #Not required, but serves as a convenient source of log entries
        manager = Manager()

        print("\nDemonstrating Source Level\n")    

        Log.enable(LogLevel.PHIDGET_LOG_INFO, "source_level_log.txt")    

        print("\nSetting source _phidget22channel to LogLevel.PHIDGET_LOG_VERBOSE\n")    

        #Knowing the list of sources, you can set a specific source to a different
        #log level than the others, to allow more or less detail for that source
        Log.setSourceLevel("_phidget22channel", LogLevel.PHIDGET_LOG_VERBOSE)    

        print("\nOpening Manager: Plug and unplug Phidgets, then check source_level_log.txt to\nsee what happened.\n")    

        manager.open()   

        time.sleep(5)    

        print("\nClosing Manager\n")    

        manager.close()  

        time.sleep(1)    

        Log.disable()    

        time.sleep(10)
        
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)   

main() #Run the main example             
