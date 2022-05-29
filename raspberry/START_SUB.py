#!/usr/bin/python3

# Author: Theodor Giles
# Created: 7/14/20
# Last Edited 5/18/22
# Description:
# This program is very basic, just so we can mess with a variable or two. will be integrated into gui?
#

import RPi.GPIO as GPIO
import time

buzzer = 26
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzer, GPIO.OUT)

def run():
    from task_io_lib_v2 import TaskIO

    print(' ===== -ROVERSUB- v3.0 ===== ')

    print("Button pushed... Starting up...")
    # time.sleep(3)
    for i in range(5):
        GPIO.output(buzzer, GPIO.HIGH)
        time.sleep(0.4)
        GPIO.output(buzzer, GPIO.LOW)
        time.sleep(0.2)

    # self, mission filename, usingarduino, usingvision, usinggyro, usingsim, usingping
    Mission = TaskIO("mission.txt", True, False, True, False, True)
    # testing funcs

    Mission.testData()

    # actual task/mission runner
    # Mission.get_tasks()
    Mission.terminate()
    print("done")


run()
