#!python3
# Author: Theodor Giles
# Created: 7/17/22
# Last Edited 7/18/22
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
from threading import Thread

# from matplotlib import pyplot as plt

X: int = 0
Y: int = 1


# Define VideoStream class to handle streaming of video from webcam in separate processing thread
# Source - Adrian Rosebrock, PyImageSearch: https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/

class VideoStream:
    """Camera object that controls video streaming from the Picamera"""

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
    floating_model = True
    interpreter = 0
    input_details = []
    output_details = []
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

    def __init__(self, left=0, right=1, show_images=True, resolution='640x480', graph='model.tflite',
                 labelmap_name='labelmap.txt', threshold=0.5, edgetpu=False, model_dir=""):
        import tensorflow as tf
        # globals

        # in mm
        FOCALLENGTH = 4.8

        # in mm
        # global CELLPHONEXSIZE
        # global TARGETYSIZE
        # CELLPHONEXSIZE = 68.2
        # TARGETYSIZE = 145.6

        graph_def_file = model_dir + "/vision/saved_model1.pb"

        input_arrays = ["Input"]
        output_arrays = ["output"]

        converter = tf.contrib.lite.TocoConverter.from_frozen_graph(graph_def_file, input_arrays, output_arrays)

        tflite_model = converter.convert()
        open("converted_model.tflite", "wb").write(tflite_model)

        MODEL_NAME = 'model.tflite'
        self.SHOW_IMAGES = show_images
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
            self.interpreter = Interpreter(model_path=PATH_TO_CKPT,
                                           experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
            print(PATH_TO_CKPT)
        else:
            self.interpreter = Interpreter(model_path=PATH_TO_CKPT)

        self.interpreter.allocate_tensors()

        # Get model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

        # Initialize frame rate calculation
        self.frame_rate_calc = 1
        self.freq = cv2.getTickFrequency()

        # Initialize video stream
        self.videostream = VideoStream(resolution=(imageWidth, imageHeight), framerate=30).start()
        print("Starting Stereo Stream...")
        self.right_cam_index = right
        self.left_cam_index = left
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
        frame1 = self.videostream.read()

        retL, imgL = self.getImg(0)
        retR, imgR = self.getImg(1)

        stereo = cv2.StereoBM(1, 16, 15)
        disparity = stereo.compute(imgL, imgR)

        # plt.imshow(disparity, 'gray')
        # plt.show()
        # Acquire frame and resize to expected shape [1xHxWx3]
        frame = imgL.copy()
        frame.getCvFrame()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (self.width, self.height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Retrieve detection results
        BoxesTF = self.interpreter.get_tensor(self.output_details[0]['index'])[
            0]  # Bounding box coordinates of detected objects
        ClassesTF = self.interpreter.get_tensor(self.output_details[1]['index'])[0]  # Class index of detected objects
        ScoresTF = self.interpreter.get_tensor(self.output_details[2]['index'])[0]  # Confidence of detected objects
        # num = interpreter.get_tensor(output_details[3]['index'])[0]
        # Total number of detected objects

        # checking for specific target
        # if searchingfor is not None:
        #     LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, \
        #         FoundTarget = process_distance_from(BoxesTF, ClassesTF, ScoresTF, searchingfor)

        return LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, FoundTarget

    def getImg(self, camindex):
        self.ret, self.img = self.Captures[camindex].read()
        return self.ret, self.img

    # def getColorMaskContours(self, ret, img, extratelem=False):
    #     # red values 179, 255,255
    #     # min 105 0 0
    #     hmin = 105
    #     smin = 0
    #     vmin = 0
    #
    #     hmax = 179
    #     smax = 255
    #     vmax = 255
    #
    #     # masking bounds
    #     x = y = 30
    #     w = h = 400
    #
    #     # contours
    #     con = False
    #
    #     if ret:
    #         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #         lower = np.array([hmin, smin, vmin])
    #         upper = np.array([hmax, smax, vmax])
    #         mask = cv2.inRange(hsv, lower, upper)
    #         # masked = cv2.bitwise_and(hsv,hsv,mask=mask)
    #         con = cv2.findContours(mask.copy(),
    #                                cv2.RETR_EXTERNAL,
    #                                cv2.CHAIN_APPROX_SIMPLE)[-2]
    #         while extratelem:
    #             if len(con) > 0:
    #                 i = 0
    #                 for c in con:
    #                     area = cv2.contourArea(c)
    #                     if area > 20:
    #                         (x, y, w, h) = cv2.boundingRect(c)
    #                         cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 255, 255), 2)
    #                         # the center fo the screen will half the resoltion hight and half the width
    #                         # then just store the x and y components
    #                         print('element:', i)
    #                         print("x:", x)
    #             cv2.imshow("result", self.img)
    #             cv2.imshow("masked", mask)
    #             # trak bars for other stuff
    #             if cv2.waitKey(1) == ord('q'):
    #                 break
    #     return con

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

    def process_distance_from(self, boxes, classes, scores, searchingfor):
        OffCenterX = 0
        OffCenterY = 0
        LateralDistance = 0.0
        Distance = 0.0
        FoundTarget = False
        for i in range(len(scores)):
            if (scores[i] > self.min_conf_threshold) and (scores[i] <= 1.0):
                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image
                # dimensions, need to force them to be within image using max() and min()
                MinY = int(max(1, (boxes[i][0] * self.imageHeight)))
                MinX = int(max(1, (boxes[i][1] * self.imageWidth)))
                MaxY = int(min(self.imageHeight, (boxes[i][2] * self.imageHeight)))
                MaxX = int(min(self.imageWidth, (boxes[i][3] * self.imageWidth)))

                object_name = self.LabelsTF[
                    int(classes[i])]  # Look up object name from "labels" array using class index
                if object_name == searchingfor:
                    FoundTarget = True
                    # LateralDistance, Distance, OffCenterX, OffCenterY = findDistance(MaxX, MinX, MaxY, MinY)
                    break
        return LateralDistance, Distance, OffCenterX, OffCenterY, FoundTarget

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
        cv2.destroyAllWindows()
        self.videostream.stop()

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
