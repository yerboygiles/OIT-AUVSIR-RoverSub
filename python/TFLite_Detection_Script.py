#!python3
######## Webcam Object Detection Using Tensorflow-trained Classifier #########
#
# Author: Evan Juras
# Date: 9/28/19
# Description: 
# This program uses a TensorFlow Lite object detection model to perform object 
# detection on an image or a folder full of images. It draws boxes and scores 
# around the objects of interest in each image.
#
# This code is based off the TensorFlow Lite image classification example at:
# https://github.com/tensorflow/tensorflow/blob/master/tensorflow/lite/examples/python/label_image.py
#
# I added my own method of drawing boxes and labels using OpenCV.
#
# ----------------------------------------------------------------------------
# 
# Modified By: Dillon Wall
# dillon.wall@oit.edu
# Date: 4/26/2020
#
#
# We can use the C++ Python API to embed a python interpreter into C++ which will allow us to get the return value
# from a specific python function, and therefore run inference on single images using this script
#
# I refactored the code to be completely wrapped in functions so that we can call image processing on demand 
#       to evaluate an image and get a value returned to C++
#
# Modified By: Theodor Giles
# farmergilest@outlook.com
# First Modified: 7/14/20
# Last Edited 8/3/20
#
#
# Distance calculation and integration into other programs/classes/handlers for running the RoboSub
# Some standardization and future algorithms possibly not using the Tensorflow AI
#

# use newer python

# Import packages *********************************************************************************
import os
# import argparse
# from typing import List
import math
import cv2
import numpy as np
import sys
import time
from threading import Thread
import importlib.util
import tensorflow as tf


# Define VideoStream class to handle streaming of video from webcam in separate processing thread
# Source - Adrian Rosebrock, PyImageSearch: https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/
class VideoStream:
    """Camera object that controls video streaming from the Picamera"""

    def __init__(self, resolution=(640, 480), framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
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


# *************************************************************************************************
# Initialize environment variables and set things up for single images to be able to be processed.
# Returns: None
# Thread Safety: None
# *************************************************************************************************
def init(model_dir, show_images=False, resolution='640x480', graph='model.tflite', labelmap_name='labelmap.txt',
         threshold=0.5, edgetpu=False):
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

    graph_def_file = model_dir+"/saved_model/saved_model.pb"

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
    videostream = VideoStream(resolution=(imageWidth, imageHeight), framerate=30).start()
    time.sleep(1)


# *************************************************************************************************

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
def process_image(searchingfor=None):
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

    print('Error! No image found...')
    finalize()


# *************************************************************************************************

# *************************************************************************************************
# This function uses the OpenCV library to show the image specified with its corresponding bounding
#   boxes, classes, and scores shown on the image.
# This takes in the image, image height, image width, bounding box coordinate list, class list, and
#   score list.
# Returns: None
# Thread Safety: None
# *************************************************************************************************
def draw_detected_frame(frame, imageheight, imagewidth, boxes, classes, scores):
    global frame_rate_calc
    # Loop over all detections and draw detection box if confidence is above minimum threshold
    cv2.namedWindow('Object Detector')

    for i in range(len(scores)):
        if (scores[i] > min_conf_threshold) and (scores[i] <= 1.0):
            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image
            # dimensions, need to force them to be within image using max() and min()
            ymin = int(max(1, (boxes[i][0] * imageheight)))
            xmin = int(max(1, (boxes[i][1] * imagewidth)))
            ymax = int(min(imageheight, (boxes[i][2] * imageheight)))
            xmax = int(min(imagewidth, (boxes[i][3] * imagewidth)))

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)

            object_name = LabelsTF[int(classes[i])]  # Look up object name from "labels" array using class index
            # drawing distance between searchingobject
            # Draw label
            label = '%s: %d%%' % (object_name, int(scores[i] * 100))  # Example: 'person: 72%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Get font size
            label_ymin = max(ymin, labelSize[1] + 10)  # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10),
                          (xmin + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255),
                          cv2.FILLED)  # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0),
                        2)  # Draw label text

    # Draw framerate in corner of frame
    cv2.putText(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2,
                cv2.LINE_AA)

    # All the results have been drawn on the frame, so it's time to display it.
    cv2.imshow('Object Detector', frame)
    # NOT IN A LOOP ANYMORE, JUST SHOW IMAGES AND LET USER CALL FINALIZE TO CLOSE
    # --# Press any key to continue to next image, or press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        finalize()
    # Calculate framerate
    t2 = cv2.getTickCount()
    time1 = (t2 - t1) / freq
    frame_rate_calc = 1 / time1


def process_distance_from(boxes, classes, scores, searchingfor):
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
                LateralDistance, Distance, OffCenterX, OffCenterY = distance3(MaxX, MinX, MaxY, MinY)
                break
    return LateralDistance, Distance, OffCenterX, OffCenterY, FoundTarget


# distance function ergh, doesn't quite work. so close
# def distance1(maxx, minx, maxy, miny):
#     # my math is horrid but it works
#     # To find the CONSTANT_distancetocm variable, I basically had to manually measure
#     # distance of objects from the camera, then divided the distance from the camera with a number gained
#     # from the equation (m/(1-(a/b))) m - distance between when a and b were measured, a -
#     SizeX = (maxx - minx)
#     SizeY = (maxy - miny)
#     pixeltomm = 145.6 / SizeY
#     ObjectCenterX = (minx - maxx) / 2
#     CONSTANT_distancetocm = 698
#     OffCenterX = int(maxx - (SizeX / 2)) - int(imageWidth / 2)
#     OffCenterY = int(maxy - (SizeY / 2)) - int(imageHeight / 2)
#     DistanceC = math.sqrt(pow(OffCenterX, 2) + pow(OffCenterY, 2))
#     DistanceA = ((CELLPHONEYSIZE * FOCALLENGTH) / SizeY)
#     DistanceB = math.sqrt(pow(DistanceA, 2) + pow(DistanceC, 2))
#     Distance = ((CELLPHONEYSIZE * FOCALLENGTH) / SizeY)
#     return LateralDistance, Distance, OffCenterX, OffCenterY
#
# not being used, maybe make another equation later on
# def distance2(maxx, minx, maxy, miny):
#     SizeX = (maxx - minx)
#     SizeY = (maxy - miny)
#     OffCenterX = int(maxx - (SizeX / 2)) - int(imageWidth / 2)
#     OffCenterY = int(maxy - (SizeY / 2)) - int(imageHeight / 2)
#     magnification = CELLPHONEYSIZE / (maxy - miny)
#     Distance = FOCALLENGTH / magnification
#     DistanceMM = Distance / 25.4
#     return LateralDistance, Distance, OffCenterX, OffCenterY
#

def distance3(maxx, minx, maxy, miny):
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


def finalize():
    # Clean up
    cv2.destroyAllWindows()
    videostream.stop()
    sys.exit()
