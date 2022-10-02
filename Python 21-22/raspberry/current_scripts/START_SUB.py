#!/usr/bin/python3

# Author: Theodor Giles
# Created: 7/14/20
# Last Edited 7/23/22
# Description:
# This program is very basic, just so we can mess with a variable or two. will be integrated into gui?
#

import time

# buzzer = 26
# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(buzzer, GPIO.OUT)

from task_io_lib_v2 import TaskIO

# def run_Nard_Nvis_Ngyr_Nsim_Npng():
#     pass
# def run_Yard_Nvis_Ngyr_Nsim_Npng():
#     pass
# def run_Yard_Yvis_Ngyr_Nsim_Npng():
#     pass


def run():
    # time.sleep(3)
    # for i in range(5):
    #     GPIO.output(buzzer, GPIO.HIGH)
    #     time.sleep(0.4)
    #     GPIO.output(buzzer, GPIO.LOW)
    #     time.sleep(0.2)

    # self, mission filename, usingvision, usinggyro, usingsim, usingsonar
    Mission = TaskIO("../mission.txt", False, True, False, False)

    # testing funcs
    # Mission.testData()
    # autonomous
    print("Running auto state")
    Mission.autonomousStMch_alt()

    # try:
    #     # actual task/mission runner
    #     Mission.get_tasks()
    # except KeyboardInterrupt:
    #     print("Interrupt. Ctrl-C'ed out.")
    #     Mission.terminate()

    Mission.terminate()
    print("done")


print(' ===== -ROVERSUB- v3.0 ===== ')

print("Running main.")
# main fn
run()
