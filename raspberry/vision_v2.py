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

    left_cam_index = 0
    right_cam_index = 1

    def __init__(self, left=0, right=1, show_images=False, resolution='640x480', graph='model.tflite',
                 labelmap_name='labelmap.txt', threshold=0.5, edgetpu=False, model_dir=""):
        # globals
        global CWD_PATH
        global width
        global height
        global floating_model
        global interpreter
        global input_details
        global output_details
        global frame_rate_calc
        global freq
        global videostream
        global t1
        global t2
        global time1
        global imageWidth
        global imageHeight
        global min_conf_threshold
        global frame_rate_calc
        global SHOW_IMAGES
        global LabelsTF

        global FOCALLENGTH
        # in mm
        FOCALLENGTH = 4.8

        # in mm
        global CELLPHONEXSIZE
        global TARGETYSIZE
        CELLPHONEXSIZE = 68.2
        TARGETYSIZE = 145.6

        graph_def_file = model_dir + "/vision/saved_model1.pb"

        input_arrays = ["Input"]
        output_arrays = ["output"]

        converter = tf.contrib.lite.TocoConverter.from_frozen_graph(graph_def_file, input_arrays, output_arrays)

        tflite_model = converter.convert()
        open("converted_model.tflite", "wb").write(tflite_model)

        MODEL_NAME = 'model.tflite'
        SHOW_IMAGES = show_images
        GRAPH_NAME = graph
        LABELMAP_NAME = labelmap_name
        min_conf_threshold = float(threshold)
        use_TPU = edgetpu
        resW, resH = resolution.split('x')
        imageWidth, imageHeight = int(resW), int(resH)

        # Import TensorFlow libraries
        # If tensorflow is not installed, import interpreter from tflite_runtime, else import from regular tensorflow
        # If using Coral Edge TPU, import the load_delegate library
        pkg = importlib.util.find_spec('tensorflow')
        if pkg is None:
            from tflite_runtime.interpreter import Interpreter
            if use_TPU:
                from tflite_runtime.interpreter import load_delegate
        else:
            from tensorflow.lite.python.interpreter import Interpreter
            if use_TPU:
                from tensorflow.lite.python.interpreter import load_delegate

        # If using Edge TPU, assign filename for Edge TPU model
        if use_TPU:
            # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
            if (GRAPH_NAME == 'detect.tflite'):
                GRAPH_NAME = 'edgetpu.tflite'

        # Get path to current working directory
        CWD_PATH = os.getcwd()

        # Path to .tflite file, which contains the model that is used for object detection
        PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)

        # Path to label map file
        PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, LABELMAP_NAME)

        # Load the label map
        with open(PATH_TO_LABELS, 'r') as f:
            LabelsTF = [line.strip() for line in f.readlines()]

        # Have to do a weird fix for label map if using the COCO "starter model" from
        # https://www.tensorflow.org/lite/models/object_detection/overview
        # First label is '???', which has to be removed.
        if LabelsTF[0] == '???':
            del (LabelsTF[0])

        # Load the Tensorflow Lite model.
        # If using Edge TPU, use special load_delegate argument
        if use_TPU:
            interpreter = Interpreter(model_path=PATH_TO_CKPT,
                                      experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
            print(PATH_TO_CKPT)
        else:
            interpreter = Interpreter(model_path=PATH_TO_CKPT)

        interpreter.allocate_tensors()

        # Get model details
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        height = input_details[0]['shape'][1]
        width = input_details[0]['shape'][2]

        floating_model = (input_details[0]['dtype'] == np.float32)

        # Initialize frame rate calculation
        frame_rate_calc = 1
        freq = cv2.getTickFrequency()

        # Initialize video stream

        # videostream = VideoStream(resolution=(imageWidth, imageHeight), framerate=30).start()
        print("Starting Stereo Stream...")
        self.right_cam_index = right
        self.left_cam_index = left
        i = 0
        while i < 2:
            self.Captures.append(cv2.VideoCapture(i))
            i = i + 1
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
        global t1
        OffCenterX = 0
        OffCenterY = 0
        LateralDistanceMM = 0.0
        DistanceMM = 0.0
        FoundTarget = False
        input_mean = 127.5
        input_std = 127.5
        # Start timer (for calculating frame rate)
        t1 = cv2.getTickCount()

        # Grab frame from video stream
        frame1 = videostream.read()

        retL, imgL = self.getImg(0)
        retR, imgR = self.getImg(1)

        stereo = cv2.StereoBM(1, 16, 15)
        disparity = stereo.compute(imgL, imgR)

        plt.imshow(disparity, 'gray')
        plt.show()
        # Acquire frame and resize to expected shape [1xHxWx3]
        frame = frame1.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()

        # Retrieve detection results
        BoxesTF = interpreter.get_tensor(output_details[0]['index'])[0]  # Bounding box coordinates of detected objects
        ClassesTF = interpreter.get_tensor(output_details[1]['index'])[0]  # Class index of detected objects
        ScoresTF = interpreter.get_tensor(output_details[2]['index'])[0]  # Confidence of detected objects
        # num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and not needed)

        if SHOW_IMAGES:
            draw_detected_frame(frame, imageHeight, imageWidth, BoxesTF, ClassesTF, ScoresTF)

        # checking for specific target
        if searchingfor is not None:
            LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, \
            FoundTarget = process_distance_from(BoxesTF, ClassesTF, ScoresTF, searchingfor)

        return LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, FoundTarget

    def getImg(self, camindex):
        self.ret, self.img = self.Captures[camindex].read()
        return self.ret, self.img

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

        self.getImg(self.right_cam_index)

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
        ret_left, img_left = self.Captures[self.left_cam_index].read()
        ret_right, img_right = self.Captures[self.left_cam_index].read()
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

    def process_distance_from(self, boxes, classes, scores, searchingfor):
        OffCenterX = 0
        OffCenterY = 0
        LateralDistance = 0.0
        Distance = 0.0
        FoundTarget = False
        for i in range(len(scores)):
            if (scores[i] > min_conf_threshold) and (scores[i] <= 1.0):
                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image
                # dimensions, need to force them to be within image using max() and min()
                MinY = int(max(1, (boxes[i][0] * imageHeight)))
                MinX = int(max(1, (boxes[i][1] * imageWidth)))
                MaxY = int(min(imageHeight, (boxes[i][2] * imageHeight)))
                MaxX = int(min(imageWidth, (boxes[i][3] * imageWidth)))

                object_name = LabelsTF[int(classes[i])]  # Look up object name from "labels" array using class index
                if object_name == searchingfor:
                    FoundTarget = True
                    LateralDistance, Distance, OffCenterX, OffCenterY = findDistance(MaxX, MinX, MaxY, MinY)
                    break
        return LateralDistance, Distance, OffCenterX, OffCenterY, FoundTarget

    def findDistance(self,maxx, minx, maxy, miny):
        THEODORS_NUMBER = 0.0516657316
        SizeX = (maxx - minx)
        SizeY = (maxy - miny)
        OffCenterX = int(maxx - (SizeX / 2)) - int(imageWidth / 2)
        OffCenterY = int(maxy - (SizeY / 2)) - int(imageHeight / 2)
        # lateral distance of camera from object
        LateralDistance = (TARGETYSIZE * FOCALLENGTH) / SizeY
        LateralDistance = (LateralDistance / THEODORS_NUMBER) * 10
        # a side of mm travel laterally triangle

        TargetPX = SizeY / 2
        TargetMM = TARGETYSIZE
        # ratio of pixel to mm
        MM__PX = TargetPX / TargetMM
        # b side of mm travel laterally triangle
        Bpx = math.sqrt(pow(OffCenterX, 2) + pow(OffCenterY, 2))
        Bmm = Bpx * MM__PX
        # c side of mm travel laterally triangle
        # true exact distance of camera from object, no matter
        # where it is on the plane
        Distance = math.sqrt(pow(Bmm, 2) + pow(LateralDistance, 2))

        return LateralDistance, Distance, OffCenterX, OffCenterY

    def terminate(self):
        # Clean up
        cv2.destroyAllWindows()
        videostream.stop()
        sys.exit()
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
