#!/usr/bin/python3

# Author: Theodor Giles
# Created: 7/14/20
# Last Edited 7/27/20
# Description:
# This program is very basic, just so we can mess with a variable or two. will be integrated into gui?
#
def run():
    from task_io_lib_v2 import TaskIO

    print(' ===== -ROVERSUB- v3.0 ===== ')

    Mission = TaskIO("mission.txt", False, False, True, False)
    while True:
        Mission.testData()
    # Mission.get_tasks(True)
    Mission.terminate()
    print("done")


run()