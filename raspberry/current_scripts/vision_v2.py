#!python3
# Author: Theodor Giles, JayCe Leonard
# Created: 7/17/22
# Last Edited 7/24/22
# Description:
# node for moving around data from the vision
# processing system

# used github references/code -
# https://github.com/niconielsen32/ComputerVision (the stereo subdirecs)
# https://github.com/AndreYonadam/OpenCV-stereo-vision
# https://github.com/aliyasineser/stereoDepth
# https://github.com/nicknochnack/TFODCourse

# import serial
# import re
# import math
# import argparse
# from math import *
import time
import os
import cv2
import numpy as np
from threading import Thread
import tensorflow as tf

# from matplotlib import pyplot as plt

X: int = 0
Y: int = 1


# info about base videostream class -
# Define VideoStream class to handle streaming of video from webcam in separate processing thread Source - Adrian
# Rosebrock, PyImageSearch: https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and
# -opencv/

class VideoStream:

    def __init__(self, resolution=(640, 480), framerate=30, camindex=0):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(camindex)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3, resolution[0])
        ret = self.stream.set(4, resolution[1])

        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

        # Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
        # Start the thread that reads frames from the video stream
        self.thread = Thread(target=self.stopped, args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # Return the most recent frame
        return self.frame

    def stop(self):
        # Indicate that the camera and thread should be stopped
        self.stopped = True


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
    left_cam_index = 0
    right_cam_index = 1
    CWD_PATH = ""
    width: float
    height: float
    frame_rate_calc = 0
    freq = 0
    VideostreamL = VideoStream
    VideostreamR = VideoStream
    videostream = VideoStream
    t1: float
    t2: float
    imageWidth: int
    imageHeight: int
    min_conf_threshold: int
    SHOW_IMAGES: bool
    LabelsTF = []
    FOCALLENGTH: float
    SeenTarget: bool

    def __init__(self, left=0, right=1, show_images=True, resolution='640x480', graph='model.tflite',
                 labelmap_name='labelmap.txt', threshold=0.5, edgetpu=False, model_dir=""):
        import tensorflow as tf

        # in mm
        FOCALLENGTH = 4.8

        MODEL_PATH = "D:/Desktop/AUVSIR_21-22/OIT-AUVSIR-RoverSub/raspberry/vision_testing/StereoVision_2" \
                     "/stereoVisionCalibration/models/Tensorflow/data/models/robosub1/saved_model "

        self.model = tf.saved_model.load(MODEL_PATH)
        self.infer = self.model.signatures["serving_default"]
        print("infer: ", self.infer.structured_outputs)

        self.min_conf_threshold = float(threshold)

        # Initialize frame rate calculation
        self.frame_rate_calc = 1
        self.freq = cv2.getTickFrequency()

        resW, resH = resolution.split('x')
        imageWidth, imageHeight = int(resW), int(resH)

        cv_file = cv2.FileStorage()
        cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

        self.stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
        self.stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
        self.stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
        self.stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

        # Initialize video stream
        print("Starting Stereo Stream...")
        self.VideostreamL = VideoStream(resolution=(imageWidth, imageHeight), framerate=30, camindex=left).start()
        self.VideostreamR = VideoStream(resolution=(imageWidth, imageHeight), framerate=30, camindex=right).start()
        time.sleep(1)

    # *************************************************************************************************
    # Processes a single image with the specified file name
    # Returns: A tuple of 3 lists:
    #                   + boundingBoxes     : list (python array) of arrays size 4.
    #                                           These size 4 arrays have top,bottom,left,right coordinates respectively
    #                                           (I think they are percent based, so 0 to 1)
    #                   + classifications   : list (python array) which contains strings of what TensorFlow classified the object as
    #                   + accuracies        : list (python array) which contains accuracies (out of 100.0%) of how "certain" the model thinks it is.
    #                                           It has a percentage for each classification totaling to 100%, but only outputs its most accurate prediction
    #                                           (Also, I believe it doesnt display anything if the prediction is below 50% accurate)
    #
    #                   Note: These three arguments should be exactly the same length, and correspond exactly to each other with index
    # Thread Safety: None
    # *************************************************************************************************
    def process_image(self, searchingfor=None):
        # global t1
        OffCenterX = 0
        OffCenterY = 0
        LateralDistanceMM = 0.0
        DistanceMM = 0.0
        FoundTarget = False
        input_mean = 127.5
        input_std = 127.5
        # Start timer (for calculating frame rate)
        self.t1 = cv2.getTickCount()
        # Grab frame from video stream
        retL, imgL = self.VideostreamL.read()
        retR, imgR = self.VideostreamR.read()

        stereo = cv2.StereoBM(1, 16, 15)
        disparity = stereo.compute(imgL, imgR)

        # plt.imshow(disparity, 'gray')
        # plt.show()
        # Acquire frame and resize to expected shape [1xHxWx3]

        infer = self.model.signatures["serving_default"]
        output_details = infer.structured_outputs

        image_np = np.asarray(np.array(imgL))
        input_tensor = tf.convert_to_tensor(image_np)
        input_tensor = input_tensor[tf.newaxis, ...]
        input_tensor = input_tensor[:, :, :, :3]  # <= add this line
        # Retrieve detection results
        BoxesTF = self.infer(input_tensor)["detection_boxes"][0].numpy()
        ClassesTF = self.infer(input_tensor)["detection_classes"][0].numpy()
        ScoresTF = self.infer(input_tensor)["detection_scores"][0].numpy()

        self.process_tensor(imgL, BoxesTF, ClassesTF, ScoresTF)
        # checking for specific target
        # if searchingfor is not None:
        #     LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, \
        #         FoundTarget = process_distance_from(BoxesTF, ClassesTF, ScoresTF, searchingfor)

        return LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, FoundTarget

    def getImg(self, camindex):
        self.ret, self.img = self.Captures[camindex].read()
        return self.ret, self.img

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
        self.img = self.VideostreamL.read()
        # self.getImg(self.right_cam_index)

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
        ret_right, img_right = self.VideostreamR.read()
        ret_left, img_left = self.VideostreamL.read()

        img_right_rm = cv2.remap(img_right, self.stereoMapR_x, self.stereoMapR_y, cv2.INTER_LANCZOS4,
                                 cv2.BORDER_CONSTANT, 0)
        frame_left_rm = cv2.remap(img_left, self.stereoMapL_x, self.stereoMapL_y, cv2.INTER_LANCZOS4,
                                  cv2.BORDER_CONSTANT,
                                  0)

        gray_right = cv2.cvtColor(img_right_rm, cv2.COLOR_BGR2GRAY)
        gray_left = cv2.cvtColor(frame_left_rm, cv2.COLOR_BGR2GRAY)

        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(gray_left, gray_right)
        # if showim:
        #     plt.imshow(disparity, 'gray')
        #     plt.show()
        return disparity

    # this should create a second object confirmer that can also use the depth map
    # by creating midpoints from both left/right contour coordinates
    def ColorStereoTarget(self, pxlim=10):
        # x offset, y offset, width of target, height of target, area of target
        # x y w h a
        SeenObjects = []
        ret_left, img_left = self.Captures[self.left_cam_index].read()
        ret_right, img_right = self.Captures[self.left_cam_index].read()
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

    def getColorMaskContours(self, ret, img, extratelem=False):
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
            while extratelem:
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

    def process_tensor(self, boxes, classes, scores, searchingfor=2, stereo=False):
        LateralDistance = 0.0
        Distance = 0.0
        self.SeenTarget = False
        for i in range(len(scores)):
            if (scores[i] > self.min_conf_threshold) and (scores[i] <= 1.0):
                self.SeenTarget = True
                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image
                # dimensions, need to force them to be within image using max() and min()

                # image center coordinates subtracted by detected box coordinates - difference in detected object
                # position in image and the vision origin/"middle" of image
                object_name = int(classes[i])
                if object_name == searchingfor:
                    MinY = int(max(1, (boxes[i][0] * self.imageHeight)))
                    MinX = int(max(1, (boxes[i][1] * self.imageWidth)))
                    MaxY = int(min(self.imageHeight, (boxes[i][2] * self.imageHeight)))
                    MaxX = int(min(self.imageWidth, (boxes[i][3] * self.imageWidth)))
                    self.SeenTarget = True
                    self.Error[Y] = (self.imageHeight / 2) - MaxY - (MaxY - MinY) / 2
                    self.Error[X] = (self.imageWidth / 2) - MaxX - (MaxX - MinX) / 2
                    self.CalculateError()
                    # LateralDistance, Distance, OffCenterX, OffCenterY = findDistance(MaxX, MinX, MaxY, MinY)
                else:
                    self.SeenTarget = False

        t2 = cv2.getTickCount()
        time1 = (t2 - self.t1) / self.freq
        frame_rate_calc = 1 / time1
        return LateralDistance, Distance, self.Error[X], self.Error[Y], FoundTarget, self.SeenTarget

        # old function for finding size of a test object.
        # need to repurpose into system for calibrating camera distance algo.

    # def findDistance(self, maxx, minx, maxy, miny):
    #     THEODORS_NUMBER = 0.0516657316
    #     SizeX = (maxx - minx)
    #     SizeY = (maxy - miny)
    #     OffCenterX = int(maxx - (SizeX / 2)) - int(imageWidth / 2)
    #     OffCenterY = int(maxy - (SizeY / 2)) - int(imageHeight / 2)
    #     # lateral distance of camera from object
    #     LateralDistance = (TARGETYSIZE * FOCALLENGTH) / SizeY
    #     LateralDistance = (LateralDistance / THEODORS_NUMBER) * 10
    #     # a side of mm travel laterally triangle
    #
    #     TargetPX = SizeY / 2
    #     TargetMM = TARGETYSIZE
    #     # ratio of pixel to mm
    #     MM__PX = TargetPX / TargetMM
    #     # b side of mm travel laterally triangle
    #     Bpx = math.sqrt(pow(OffCenterX, 2) + pow(OffCenterY, 2))
    #     Bmm = Bpx * MM__PX
    #     # c side of mm travel laterally triangle
    #     # true exact distance of camera from object, no matter
    #     # where it is on the plane
    #     Distance = math.sqrt(pow(Bmm, 2) + pow(LateralDistance, 2))
    #
    #     return LateralDistance, Distance, OffCenterX, OffCenterY

    def terminate(self):
        # Clean up
        # video telem
        # out.release()
        cv2.destroyAllWindows()
        self.videostream.stop()

    def CalculateError(self, ):

        # previous error for error delta
        # gyro
        self.Previous_Error[X] = self.Error[X]
        self.Previous_Error[Y] = self.Error[Y]

        # sum of error for integral
        # gyro
        self.Error_Sum[X] = self.Error_Sum[X] + self.Error[X]
        self.Error_Sum[Y] = self.Error_Sum[Y] + self.Error[Y]
        # math for change in error to do derivative
        # gyro
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
