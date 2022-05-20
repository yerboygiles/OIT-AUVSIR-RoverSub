#!python3
# Author: Theodor Giles
# Created: 5/21/21
# Last Edited 5/26/21
# Description:
# REMOTE CONTROL! probably will put this into the commander at some point, but it's nice to have as like a random input


def get_wasdqerv_directional() -> int:
    directval = 0
    charcommand = input("MOVE: ")
    # MOVING FORWARDS
    if charcommand == "w":
        directval = 1
    # STRAFING LEFT
    elif charcommand == "a":
        directval = 2
    # MOVING BACKWARDS
    elif charcommand == "s":
        directval = 3
    # STRAFING RIGHT
    elif charcommand == "d":
        directval = 4
    # TURNING LEFT
    elif charcommand == "q":
        directval = 5
    # TURNING RIGHT
    elif charcommand == "e":
        directval = 6
    # ASCENDING
    elif charcommand == "r":
        directval = 7
    # DESCENDING
    elif charcommand == "v":
        directval = 8
    # PAUSING
    elif charcommand == "o":
        directval = -1
    # STOPPING
    elif charcommand == "u":
        directval = -2
    #
    else:
        directval = 0
    return directval


def set_individual_motor() -> tuple[int, float]:
    directval = 0
    motor = input("MOTOR: ")
    speed = input("speed: ")
    return int(motor), float(speed)
