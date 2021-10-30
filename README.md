Welcome to the README for the Oregon Tech RoboSub Software

Written By: Theodor Giles

***Last Officially Updated On: June 16th, 2021***
   
***This readme is not finished, reading the full
   application will teach you much better.***. 

   This is all subject to constant change. There will be 
   random commits from dispersed work from me and other members at 
   varying rates.


I will go into detail on the following: 

1. **Hardware/Embedded Environment, Organization, and Cooperation**


2. **Software Environment, Structure, and Cooperation**


***I. Hardware Environment:***


**T200 Thrusters + Basic ESCs (BlueRobotics)** - 

**Arduino Mega** - Hardpoint Control, Sensor Interpretation, 
Communication

   The Arduino requires the run_on_mega_v2.ino uploaded to it, either 
   through the Rpi or another computer. This lets it communicate via 
   serial with the rPi so we can send specific commands between the two
   boards. Currently, the rPi polls the Arduino for Gyro data, and also 
   sends stop/start signals to hard reset/initialize the thrusters. It 
   also can request setting changes on the arduino for certain values, 
   such as max power, but more options are to come in the future for
   most anything that will need during-mission tuning. It can also send
   coordinates for the left/right robotic arms to set their endpoints
   to, as well as opening/closing the grippers(hands). 


**Raspberry Pi 3B+/4B+** - Mission Control, Vision, Logging, Data 
Management/Processing

   The rPi is running a much larger system than the Arduino, currently
   through Python, but our Software member Tyler Lucas is working on 
   the eventual transition to C/C++. This will make the current code 
   even lower-impact, but the Python application works great for now.
   The rPi communicates with the Arduino, stereo cameras, and an IMU.
   

**9 DOF Sensor - BN055, PhidgetSpatial 1042_0** - Gyro, Magnometer, 
Accelerometer

   We are currently using two separate IMUs, the PhidgetSpatial 1042_0, 
   and the generic adafruit BN055. The BN055 has been giving us '*good 
   enough*' data, but the PhidgetSpatial is meant to be the main IMU, 
   as it's more reliable, has its own API, and was more expensive. The 
   code running on the sub has the ability to use either independently,
   error check between two BN055s, or manage a queen/drone system between
   the PhidgetSpatial and the BN055s. As the Arduino manages BN055 data 
   interpretation, the rPi manages the PhidgetSpatial, so if one board 
   goes down or has issue communicating with its connected IMU, there
   are "backup plans" embedded into the mission control.
   

**Electrical Management/Organization** - 

   The Raspberry Pi is the brains of the entire operation, so most 
   everything will have/has a line to it. The Arduino is connected directly 
   with tx/rx wires between the Raspberry Pi. There are two voltage 
   regulators that manage 5V source going to both the Raspberry Pi and Arduino,
   respectively


***II. Software Environment***

   Pretty much all the application code is written in Python, so it is pretty 
   proficient at handling lots of advanced computation, organization, and 
   interfacing with other things, even hardware. I have the program set up 
   in a sort of tree that flows upward from a starting program to another 
   program, to another program, and branches out to other hardware and 
   software based on what is required/asked of the program. This is meant to
   provide a platform for future development that will eventually allow for
   multi-platform applications. Pretty much, I want the code to be reconfigurable
   to the extent that it can run on anything, and do anything(within reason). 
   Future projects in AUVSIR won't be just limited to submarines and rovers,
   so I want this application to be very adaptable in the future, and is also
   being worked on to translate from Python -> C++ so it can possibly run on 
   even more hardware.


   Phidget22.Phidget - (Phidget22 gyro libraries)

   Tensorflow - (AI development and interpretation)

   OpenCV - (vision processing)

   PyGame - (simulation)

   PyOpenGL - (simulation)
   
   The top-most program of our custom application is technically 
   "*START_SUB.py*", but we mainly initialize using "*START_SUB_WITH_TIME.py*",
   which is meant to give a modifiable countdown to the start, with a beeper. 

   **START_SUB** - 

   This is the most simple program in the entire application, but you can also 
   change arguments to modify the mission and it's requirements, such as 
   removing the missions ability to use the gyro, vision systems, etc. to test
   specific functions and systems in order to troubleshoot and ensure lower
   level functionality. 

```
#!python
def run():
    from task_io_lib_v2 import TaskIO

    print(' ===== ROBOSUB v2.0 ===== ')

    Mission = TaskIO("mission.txt", False, False, False)
    Mission.get_tasks()
    Mission.terminate()
    print("done")
```

*TaskIO* is the 2nd-highest program, which is initialized with *mission.txt*, 
a basic text file consisting of the commands the Sub will run, and they are 
formatted to a certain standard, which I will go into detail later. The first 
boolean argument determines if we are using our vision processing algorithms/AIs, 
the second determines the use of the PixHawk and Gyro, but I may omit the ability 
to not use the PixHawk soon because of how important the gyro data is to the 
navigation of the RoboSub. *get_tasks()* manages the *mission.txt* file and sends
it to the corresponding classes and subsystems.

   **task_io_lib_v2** -

   This program is the first level at which commands are taken in, and controls the 
   peripherals such as the gyro, vision systems, and simulation. It also initializes 
   the used *MovementCommander*.

```
#!python
class TaskIO:
    # init
        def __init__(self, filename, usingvision, usinggyro, usingsim):
        self.Filename = filename
        self.UsingVision = usingvision
        self.UsingGyro = usinggyro
        self.UsingSim = usingsim
        self.Active = False
        self.Movement = MovementCommander(self.UsingVision, self.UsingGyro, self.UsingSim)
        self.CommandList = []
```

   This is the init, and as you can see, it takes two boolean arguments if the user 
   wants to use the vision processing, simulation, and/or gyro. 

```
#!python
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

```

   This function of TaskIO is basically a run() function, but it also puts all the 
   "commands" from the *mission.txt* into a list so we don't have to parse the text 
   file in the MovementCommander(*self.Movement*) as well.
   
   **movement_commander_v4** - 

   This program is the main meat and bones of the navigation, the command interpreting, 
   thruster driving, data collecting, and I think it's the longest program in the 
   application by lines of code. I feel like I don't need to explain the *__init__()* 
   of MovementCommander, because it is self-explanatory, initializing each class and 
   hardware we want it to.

```
#!python
# handles list of commands
     def receiveCommands(self, commandlist):
        # going through commands in parsed list
        self.CommandIndex = 0
        # tell arduino to arm motors
        self.SendToArduino("STOP")
        print("Stopping arduino... Wait 3.")
        time.sleep(3)
        self.SendToArduino("START")
        print("Starting arduino... Wait 3.")
        time.sleep(3)
        self.SendToArduino("MAXPOWER:20")
        print("Sending settings... Wait 3.")
        time.sleep(3)
        try:
            for command in commandlist:
                print("VectorCommander running: ", command)
                self.MainCommand = ""
                self.SuppCommand = ""
                j = 0
                for commandParsed in str(command).split(','):
                    commandParsed.strip()
                    if j == 0:
                        self.MainCommand = commandParsed
                    if j == 1:
                        self.SuppCommand = commandParsed
                    j = j + 1
                print("Main: ", self.MainCommand, ", Supplementary: ", self.SuppCommand)
```
   The first part of receiveCommands() involves sending initialization codes to the Arduino,
   arming it, and also managing settings like MaxPower(limits motor thrust for safety, etc.).

```
#!python
                if self.MainCommand == "REMOTE":
                    print("Driver Control With:")
                    self.BasicDriverControl()
                    if self.SuppCommand == "KEYBOARD":
                        print("Keyboard!")
                    else:
                        pass
                else:
                    for basiccommand in self.BASIC_MOVEMENT_COMMANDS:
                        i = 0
                        if self.MainCommand == basiccommand:
                            self.InitialTime = time.perf_counter()
                            if self.UsingGyro:
                                self.BasicLinear(self.SuppCommand)
                            else:
                                self.BasicWithTime(self.SuppCommand)
                        i += 2
                    for advancedcommand in self.ADVANCED_MOVEMENT_COMMANDS:
                        i = 0
                        if self.MainCommand == advancedcommand:
                            self.InitialTime = time.perf_counter()
                            self.BasicVectoring(self.SuppCommand)
                        i += 2
                    self.CommandIndex += 1
```


This set of code checks the value of each command in the command list and sets the variables from before to their respective values for running the specific commands.


```
#!python
if self.IsMoveCommand:
    self.Running = True
    self.GyroRunning = True
    self.PositionRunning = True
    while self.Running:
        self.WaitForSupplementary = False
        self.UpdateThrustersPID()
        # print("Yaw: ", self.PixHawk.getYaw())
        # print("Right Back speed: ", MapToSpeed(self.ThrusterRB.GetSpeed()))
        if self.TargetOrPosition == A_GYRO:
            self.CheckIfGyroDone()
            self.Running = self.GyroRunning
        elif self.TargetOrPosition == A_POSITION:
            self.CheckIfPositionDone()
            self.Running = self.PositionRunning
        elif self.TargetOrPosition == A_TARGET:
            self.TargetCommand(self.CommandIndex, command)
        elif self.Basic:
            self.BasicCommand(self.CommandIndex)
        elif self.Advanced:
            self.AdvancedCommand(self.CommandIndex)
    self.TargetOrPosition = 0
```


This code here is for running the specific commands because once a "move command" is interpreted, it then knows to run the moving code in that command loop. It sets all the *self.Running* variables to True, so that it will run until one of them becomes false by the command ends, such as the *self.CheckIfGyroDone* that sets *self.GyroRunning* to False and also sets it's value to *self.Running* for a basic turn command, or such as the *TargetCommand()* successfully rams a target and sets *self.Running* to false. 

   **pixhawk_data** - 

   This program is dedicated to receiving and parsing Mavlink data from the Pixhawk over serial, and then allowing it's data to be accessed, as well as taking in offset values for PID calculation. 
    **!! Currently, only Proportional(the P in PID) control has been tested.  !!** 


```
#!python
GYRO: int = 0
POSITION: int = 1
YAW: int = 0
PITCH: int = 1
ROLL: int = 2
NORTH: int = 0
EAST: int = 1
DOWN: int = 2

class PixHawk:

    Gyro = [0.0, 0.0, 0.0]
    Position = [0.0, 0.0, 0.0]
    Angular_Motions = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Measures = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Previous_Error = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error_Sum = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error_Delta = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

    Kp = [[0.3, 0.4, 0.4], [0.3, 0.4, 0.4]]
    Ki = [[0.1, 0.1, 0.1], [0.1, 0.1, 0.1]]
    Kd = [[0.1, 0.1, 0.1], [0.1, 0.1, 0.1]]

    North_PID = 0.0
    North_P = 0.0
    North_I = 0.0
    North_D = 0.0

    East_PID = 0.0
    East_P = 0.0
    East_I = 0.0
    East_D = 0.0

    Down_PID = 0.0
    Down_P = 0.0
    Down_I = 0.0
    Down_D = 0.0

    Yaw_PID = 0.0
    Yaw_P = 0.0
    Yaw_I = 0.0
    Yaw_D = 0.0

    Pitch_PID = 0.0
    Pitch_P = 0.0
    Pitch_I = 0.0
    Pitch_D = 0.0

    Roll_PID = 0.0
    Roll_P = 0.0
    Roll_I = 0.0
    Roll_D = 0.0
```

All these variables are for very specific calculations, so I will walk 
through how it all comes together. First of all, the constants such as 
*GYRO: int = 0* is for accessing the Gyroscopic data from the arrays 
that are 2-dimensional. Gyro data is at 1st index 0, 2nd index 0->2, 
and position data is at 1st index 0, 2nd index 0->2. The *YAW, PITCH,*
 and *ROLL* variables are for accessing the 2nd index, so I can ensure the right data goes into the right index at any given time through readability. The *Error* variables are for PID calculations, which you can read in the program. It is pretty straight forward. The other variables are easy to interpret cause their use is in their name. 


```
#!python
i = 0
for CommaParse in str(self.vehicle.attitude).split(','):
    if CommaParse is not None:
        for EqualParse in CommaParse.split('='):
            try:
                if i == 1:
                    self.Angular_Motions[GYRO][PITCH] = (
                                self.Gyro[PITCH] - (float(EqualParse) * (180 / math.pi)))
                    self.Gyro[PITCH] = round(float(EqualParse) * (180 / math.pi), 5)
                if i == 3:
                    self.Angular_Motions[GYRO][YAW] = (
                                self.Gyro[YAW] - (float(EqualParse) * (180 / math.pi)))
                    self.Gyro[YAW] = round((float(EqualParse) * (180 / math.pi)), 5)
                if i == 5:
                    self.Angular_Motions[GYRO][ROLL] = (
                                self.Gyro[ROLL] - (float(EqualParse) * (180 / math.pi)))
                    self.Gyro[ROLL] = round(float(EqualParse) * (180 / math.pi), 5)
                i += 1
            except:
                pass
```

This is the code to parse the string data read from the Dronekit library that the Raspberry Pi uses to receive serial data from the Pixhawk, and because the data read back is more like readable telemetry, there is some string splitting that must occur, and the data is at certain points in the string, which is why the values are set to the converted strings when "i" is compared to a certain value in the for loop. I plan to improve this process at some point, making it faster, and I will update this Wiki accordingly.

**Syntax, Naming, etc.**

Currently, I use my own custom system for variable naming, syntax, etc.

CamelCase is used for most Very Important Variables. Data for 
calculation, communication, etc. Also used for more complex class 
methods.

thisCase is for basic methods like setting, getting, printing, etc.

lowercase is used for variables that are unimportant, method arguments,
or basic members. No methods.

UPPER_CASE_SPACED is mainly for constants and valuable arrays, no 
methods.

i hate using single digit variables, but when I do, it signifies an index
or counter.

This_case1 signifies a piece of hardware, the type being *This* and the 
*case1* meaning the designation and sometimes which # hardpoint it is. 
Example below. 

```
self.Gyro_drone1 = imu_ard_data.WT61P(self.serial) 

self.Thruster_VentralLB = ThrusterDriver("LB")  # left back  
```