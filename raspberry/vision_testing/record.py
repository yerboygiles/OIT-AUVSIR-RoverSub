import time
import os
import numpy as np
import cv2
import RPi.GPIO as GPIO

print
"Starting..."

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# GPIO.setup(22, GPIO.OUT)  # Red LED
# GPIO.setup(27, GPIO.OUT)  # Green LED
# GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button Input for recording
# GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Power off button

recording = False

print
"Starting OpenCV"
capture = cv2.VideoCapture(0)

imagewidth = 640
imageheight = 480
capture.set(3, imagewidth)  # 1024 640 1280 800 384
capture.set(4, imageheight)  # 600 480 960 600 288

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')

cv2.waitKey(50)


def CaptureSaveFrame(outfile):
    ret, img = capture.read()
    ret, img = capture.read()  # get a few frames to make sure current frame is the most recent
    outfile.write(img)
    cv2.waitKey(1)
    return img


#
# def LEDGreen():
#     GPIO.output(22, GPIO.HIGH)
#     GPIO.output(27, GPIO.LOW)
#
#
# def LEDRed():
#     GPIO.output(22, GPIO.LOW)
#     GPIO.output(27, GPIO.HIGH)
#
#
# def LEDOff():
#     GPIO.output(22, GPIO.HIGH)
#     GPIO.output(27, GPIO.HIGH)


def CreateFile():
    timestr = time.strftime("%Y%m%d-%H%M%S")
    print
    timestr
    out = cv2.VideoWriter('/main_software/raspberry/vision_testing/' + timestr + '.avi', fourcc, 5.0,
                          (imagewidth, imageheight))
    return out


#
# def Shutdown(channel):
#     print("Shutting Down")
#     LEDOff()
#     time.sleep(0.5)
#     LEDGreen()
#     time.sleep(0.5)
#     LEDOff()
#     time.sleep(0.5)
#     LEDGreen()
#     time.sleep(0.5)
#     LEDOff()
#     time.sleep(0.5)
#     LEDRed()
#     os.system("sudo shutdown -h now")

#
# GPIO.add_event_detect(21, GPIO.FALLING, callback=Shutdown, bouncetime=2000)
#
# LEDGreen()

InitialTime = time.perf_counter()
recording = True
out = CreateFile()
CurrentTime = time.perf_counter()
while CurrentTime - InitialTime < 10:
    CurrentTime = time.perf_counter()
    print(CurrentTime)
    CaptureSaveFrame(out)
# os.system("sudo shutdown -h now")
