#!/usr/bin/python3

# Author: Theodor Giles
# Created: 8/6/20
# Last Edited 8/7/20
# Description:
# This program is dedicated to starting up the robosub through button and buzzer peripherals on
# the raspberry pi.
#
import RPi.GPIO as GPIO
import time
import START_SUB as Mission

button = 22
buzzer = 26

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzer, GPIO.OUT)
GPIO.setup(button, GPIO.IN)

print("Press button to start...")
buttonwait = True
try:
    while buttonwait:
        buttonwait = GPIO.input(button)
except:
    pass
MissionAlive = True
#
print("Button pushed... Starting up...")
time.sleep(3)
for i in range(3):
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.01)

starttime = int(input("Seconds until start?"))
print("Starting countdown.")
for i in range(starttime):  # change this number for how many seconds you'll need to put it in the water/test
    GPIO.output(buzzer, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(buzzer, GPIO.LOW)
    time.sleep(0.01)
    time.sleep(1)
    print("T-minus: ", starttime-i)

print("Running mission...")
Mission.run()
print("Mission complete...")
print("Cleaning up...")
GPIO.cleanup()


