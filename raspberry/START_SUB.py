#!/usr/bin/python3

# Author: Theodor Giles
# Created: 7/14/20
# Last Edited 5/4/22
# Description:
# This program is very basic, just so we can mess with a variable or two. will be integrated into gui?
#
def run():
    from task_io_lib_v2 import TaskIO

    print(' ===== -ROVERSUB- v3.0 ===== ')

    Mission = TaskIO("mission.txt", True, False, True, False)
    # Mission.testArduino()
    # Mission.testData()
    Mission.get_tasks()
    Mission.terminate()
    print("done")


run()