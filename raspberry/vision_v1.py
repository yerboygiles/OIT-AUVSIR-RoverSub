#!python3
# Author: Theodor Giles
# Created: 7/13/21
# Last Edited 7/29/21
# Description:
# node for moving around data from the vision
# processing system

import serial
import time
import re
import math
import os
import argparse
import cv2
import numpy as np
from math import *
from matplotlib import pyplot as plt

X: int = 0
Y: int = 1


class vision:
    Error = [0.0, 0.0, 0.0]
    Previous_Error = [0.0, 0.0, 0.0]
    Error_Sum = [0.0, 0.0, 0.0]
    Error_Delta = [0.0, 0.0, 0.0]
    # gyro              position
    XOffset = 0.0
    YOffset = 0.0

    # this is a comment
    Kp = [0.1, 0.1, 0.0]  # constant to modify PID
    Ki = [0.1, 0.1, 0.0]  # constant to modify PID
    Kd = [0.1, 0.1, 0.0]  # constant to modify PID

    X_PID = 0.0
    X_P = 0.0
    X_I = 0.0
    X_D = 0.0

    Y_PID = 0.0
    Y_P = 0.0
    Y_I = 0.0
    Y_D = 0.0

    Captures = []

    rightcamindex = 0
    leftcamindex = 0

    def __init__(self, cameras, right, left):
        self.cameras = cameras
        self.rightcamindex = right
        self.leftcamindex = left
        i = 0
        while i < cameras:
            self.Captures.append(cv2.VideoCapture(i))
            i = i + 1

    def getImg(self, camindex):
        self.ret, self.img = self.Captures[camindex].read()
        return self.ret, self.img

    def getColorMaskContours(self, ret, img, beallfancy=False):
        # red values 179, 255,255
        # min 105 0 0
        hmin = 105
        smin = 0
        vmin = 0

        hmax = 179
        smax = 255
        vmax = 255

        # masking bounds
        x = y = 30
        w = h = 400

        # contours
        con = False

        if ret:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower = np.array([hmin, smin, vmin])
            upper = np.array([hmax, smax, vmax])
            mask = cv2.inRange(hsv, lower, upper)
            # masked = cv2.bitwise_and(hsv,hsv,mask=mask)
            con = cv2.findContours(mask.copy(),
                                   cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)[-2]
            while beallfancy:
                if len(con) > 0:
                    i = 0
                    for c in con:
                        area = cv2.contourArea(c)
                        if area > 20:
                            (x, y, w, h) = cv2.boundingRect(c)
                            cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                            # the center fo the screen will half the resoltion hight and half the width
                            # then just store the x and y components
                            print('element:', i)
                            print("x:", x)
                cv2.imshow("result", self.img)
                cv2.imshow("masked", mask)
                # trak bars for other stuff
                if cv2.waitKey(1) == ord('q'):
                    break
        return con

    def seesTargetColorMask(self, target):
        # red values 179, 255,255
        # min 105 0 0
        hmin = 105
        smin = 0
        vmin = 0

        hmax = 179
        smax = 255
        vmax = 255

        # masking bounds
        x = y = 30
        w = h = 400

        # contours
        seen = False

        self.getImg(self.rightcamindex)

        if self.ret:
            hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
            lower = np.array([hmin, smin, vmin])
            upper = np.array([hmax, smax, vmax])
            mask = cv2.inRange(hsv, lower, upper)
            # masked = cv2.bitwise_and(hsv,hsv,mask=mask)
            con = cv2.findContours(mask.copy(),
                                   cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)[-2]
            if len(con) > 0:
                i = 0
                for c in con:
                    area = cv2.contourArea(c)
                    if area > 20:
                        (x, y, w, h) = cv2.boundingRect(c)
                        seen = True
                        # cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                        # the center fo the screen will half the resoltion hight and half the width
                        # then just store the x and y components

        return seen

    def StereoTarget(self, showim):
        ret_left, img_left = self.Captures[self.leftcamindex].read()
        ret_right, img_right = self.Captures[self.leftcamindex].read()
        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(gray_left, gray_right)
        if showim:
            plt.imshow(disparity, 'gray')
            plt.show()
        return disparity

    # this should create a second object confirmer that can also use the depth map
    # by creating midpoints from both left/right contour coordinates
    def ColorStereoTarget(self, pxlim=10):
        # x offset, y offset, width of target, height of target, area of target
        # x y w h a
        SeenObjects = []
        ret_left, img_left = self.Captures[self.leftcamindex].read()
        ret_right, img_right = self.Captures[self.leftcamindex].read()
        ContoursL = self.getColorMaskContours(ret_left, img_left)
        ContoursR = self.getColorMaskContours(ret_right, img_right)
        if ContoursL and ContoursR:
            for leftcontour in ContoursL:
                leftcontour_area = cv2.contourArea(leftcontour)
                for rightcontour in ContoursR:
                    rightcontour_area = cv2.contourArea(rightcontour)
                    # area check
                    if abs(leftcontour_area - rightcontour_area) < pxlim:
                        (leftcontour_x, leftcontour_y, leftcontour_w, leftcontour_h) = cv2.boundingRect(leftcontour)
                        (rightcontour_x, rightcontour_y, rightcontour_w, rightcontour_h) = cv2.boundingRect(
                            rightcontour)
                        # width check
                        if abs(leftcontour_w - rightcontour_w) < pxlim:
                            # height check
                            if abs(leftcontour_h - rightcontour_h) < pxlim:
                                error = (abs(rightcontour_h)
                                         + abs(rightcontour_w)
                                         + abs(rightcontour_area))
                                SeenObjects.append([abs(img_right.width / 2 - rightcontour_x),
                                                    abs(img_right.height / 2 - rightcontour_y),
                                                    rightcontour_w,
                                                    rightcontour_h,
                                                    rightcontour_area
                                                    ])
        i = 0
        returnindex = 0
        for target in SeenObjects:
            if i + 1 < len(SeenObjects):
                if target[5] > SeenObjects[i + 1][5]:
                    returnindex = i + 1
            i = i + 1

        # stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        # cv2.rectangle(stereo, (SeenObjects[0], SeenObjects[1]),
        #               (SeenObjects[0] + SeenObjects[2], SeenObjects[1] + SeenObjects[3]), (0, 255, 255), 2)
        # disparity = stereo.compute(img_left, img_right)

        self.XOffset = SeenObjects[returnindex][0]
        self.YOffset = SeenObjects[returnindex][1]
        return SeenObjects[returnindex]

    # req for PID calculation
    def CalculateError(self):
        self.Error = [0, 0]
        self.Error_Sum = [0, 0]
        self.Error_Delta = [0, 0]
        self.Previous_Error = [0, 0]
        # previous error for error delta
        # gyro
        self.Previous_Error[X] = self.Error[X]
        self.Previous_Error[Y] = self.Error[Y]
        # error for proportional control
        self.Error[X] = self.XOffset
        self.Error[Y] = self.YOffset

        # sum of error for integral
        self.Error_Sum[X] = self.Error_Sum[X] + self.Error[X]
        self.Error_Sum[Y] = self.Error_Sum[Y] + self.Error[Y]

        # math for change in error to do derivative
        self.Error_Delta[X] = self.Error[X] - self.Previous_Error[X]
        self.Error_Delta[Y] = self.Error[Y] - self.Previous_Error[Y]

    # pid calculation
    def PID(self):
        # X PID variable setting
        self.X_P = (self.Error[X] * self.Kp[X])
        self.X_I = (self.Error_Sum[X] * self.Ki[X])
        self.X_D = (self.Error_Delta[X] * self.Kd[X])
        self.X_PID = self.X_P  # + self.X_I + self.X_D

        # Y PID variable setting
        self.Y_P = (self.Error[Y] * self.Kp[Y])
        self.Y_I = (self.Error_Sum[Y] * self.Ki[Y])
        self.Y_D = (self.Error_Delta[Y] * self.Kd[Y])
        self.Y_PID = self.Y_P  # + self.Y_I + self.Down_D

    def getXPID(self):
        return self.X_PID

    def getYPID(self):
        return self.Y_PID

    def getXOffset(self):
        return self.XOffset

    def getYOffset(self):
        return self.YOffset

    def getOffset(self):
        return [self.XOffset, self.YOffset]

    def getDistance(self):
        return self.Distance

    # end command/vehicle running
    def Terminate(self):
        pass

    # while True:
    #     Vision.StereoGetTarget(12)
# This is a sample Python script.
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
#
# def on_change_hmax(value):
#     global hmax
#     hmax = value
#     print (hmax)
# def on_change_smax(value):
#     global smax
#     smax = value
#     print (smax)
# def on_change_vmax(value):
#     global vmax
#     vmax = value
#     print (vmax)
# def on_change_hmin(value):
#     global hmin
#     hmin= value
#     print (hmin)
# def on_change_smin(value):
#     global smin
#     smin = value
#     print (smin)
# def on_change_vmin(value):
#     global vmin
#     vmin = value
#     print (vmin)
# name_bars = 'trackbar window'
# cv2.namedWindow('trackbar window',cv2.WINDOW_NORMAL)
# cv2.createTrackbar("hue max",name_bars,179,179,on_change_hmax)
# cv2.createTrackbar("sat max",name_bars,255,255,on_change_smax)
# cv2.createTrackbar("val max",name_bars,255,255,on_change_vmax)
# cv2.createTrackbar("hue min",name_bars,130,179,on_change_hmin)
# cv2.createTrackbar("sat min",name_bars,166,255,on_change_smin)
# cv2.createTrackbar("val min",name_bars,0,255,on_change_vmin)
# track bars
# cv2.namedWindow("parms")
# cv2.createTrackbar("thresh1","parms",88,255,empty)
# cv2.createTrackbar("thresh2","parms",20,255,empty)
# cv2.createTrackbar("thresh3","parms",88,255,empty)
# cv2.createTrackbar("thresh4","parms",20,255,empty)

# Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     #find_closest_corner()
#     #find_contour('dasub.mp4')
#      color_masking('dasub2.mp4')
#     #split_vid('dasub.mp4')
#     # show_img_func('fortesting.jpg')
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
