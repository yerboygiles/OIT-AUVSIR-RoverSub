#!python3
# Author: Theodor Giles
# Created: 8/7/20
# Last Edited 7/23/22
# Description:
# This program manages the conversion of the mission.txt into commands that the MovementCommander can understand
# as well as the AI/TF/vision integration
# Allows the movement_commander to update at all times, causing no lag for switching commands

import numpy as np
import cv2
from threading import Thread
import tensorflow as tf
from tensorflow import keras
import importlib.util
from tensorflow.lite.python.interpreter import Interpreter


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

    def isOpened(self):
        return self.stream.isOpened()

    def read(self):
        # Return the most recent frame
        return self.frame

    def stop(self):
        # Indicate that the camera and thread should be stopped
        self.stopped = True


def depth_map(imgL, imgR):
    """ Depth map calculation. Works with SGBM and WLS. Need rectified images, returns depth map ( left to right
    disparity ) """
    # SGBM Parameters -----------------
    window_size = 3  # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and
    # above); 5 Works nicely

    left_matcher = cv2.StereoSGBM_create(
        minDisparity=-1,
        numDisparities=5 * 16,  # max_disp has to be dividable by 16 f. E. HH 192, 256
        blockSize=window_size,
        P1=8 * 3 * window_size,
        # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works
        # nicely
        P2=32 * 3 * window_size,
        disp12MaxDiff=12,
        uniquenessRatio=10,
        speckleWindowSize=50,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    # FILTER Parameters
    lmbda = 80000
    sigma = 1.3
    visual_multiplier = 6

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(lmbda)

    wls_filter.setSigmaColor(sigma)
    displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
    dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
    displ = np.int16(displ)
    dispr = np.int16(dispr)
    filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!

    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=25, alpha=200, norm_type=cv2.NORM_MINMAX);
    filteredImg = np.uint8(filteredImg)

    return filteredImg


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
        if (float(scores[i]) > float(min_conf_threshold)) and (float(scores[i]) <= 1.0):
            print("Score: ", scores[i])
            print("Class: ", classes[i])
            print("Boxes: ", boxes[i])
            # print("found target...")
            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image
            # dimensions, need to force them to be within image using max() and min()
            ymin = int(max(1, (boxes[i][0] * imageheight)))
            xmin = int(max(1, (boxes[i][1] * imagewidth)))
            ymax = int(min(imageheight, (boxes[i][2] * imageheight)))
            xmax = int(min(imagewidth, (boxes[i][3] * imagewidth)))

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)
            if classes[i] == 1:
                object_name = "BUOY"
            if classes[i] == 2:
                object_name = "GATE"
            if classes[i] < 3 or classes[i] > 0:
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
                out.write(frame)
    # Draw framerate in corner of frame cv2.putText(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), (30, 50),
    # cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)

    # All the results have been drawn on the frame, so it's time to display it.
    cv2.imshow('Object Detector', frame)
    # NOT IN A LOOP ANYMORE, JUST SHOW IMAGES AND LET USER CALL FINALIZE TO CLOSE
    # --# Press any key to continue to next image, or press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        cap_left.stop()
        out.release()
        cv2.destroyAllWindows()
        # videostream.stop()


# Camera parameters to undistort and rectify images
cv_file = cv2.FileStorage()
cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

# tensorflow code

MODEL_PATH = "D:/Desktop/AUVSIR_21-22/OIT-AUVSIR-RoverSub/raspberry/vision_testing/StereoVision_2/stereoVisionCalibration/models/Tensorflow/data/models/robosub1/saved_model"
# converter = tf.lite.TFLiteConverter.from_saved_model(CWD_PATH)
#
# tflite_model = converter.convert()
# with open("model.tflite", "wb") as f:
#     f.write(tflite_model)

# SHOW_IMAGES = show_images
# GRAPH_NAME = graph
# LABELMAP_NAME = labelmap_name
min_conf_threshold = 75 / 100
# use_TPU = edgetpu
# imageWidth, imageHeight = int(resW), int(resH)
#
# model: tf.keras.Model
# actual model load
# model = tf.keras.load_model(MODEL_PATH)
# model = tf.keras.Model(MODEL_PATH)
# model.build()
# print("infer: ", infer)
# print("Structured Outputs: ", output_details)
# print("Structured Inputs: ", infer.structur)
# infer.input_tensor()
# keras.models.model_from_config()
# model.load_weights(MODEL_PATH)
# model.summary()

# PATH_TO_CKPT = os.path.join(CWD_PATH, 'model.tflite', "model.tflite")
# interpreter = Interpreter(model_path=PATH_TO_CKPT)
# interpreter.allocate_tensors()
# metrics_names = model.metrics_names  # Getting params
# print("Metric Names - ", metrics_names)
# summary = model.summary()
# print("Model Summary - ", summary)
# input_details = model.Input()
# print("Inputs - ", input_details)
# output_details = model.Output()
# print("Outputs - ", output_details)
# height = input_details[0]['shape'][1]
# width = input_details[0]['shape'][2]
# floating_model = (input_details[0]['dtype'] == np.float32)
# Open both cameras
# cap_right = VideoStream(camindex=1)
# cap_left = VideoStream(camindex=0)
# cap_right = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap_left = cv2.VideoCapture(0, cv2.CAP_DSHOW)

fourcc = cv2.VideoWriter_fourcc(*'MP4V')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))
# cap_right.start()
# cap_left.start()
input_mean = 127.5
input_std = 127.5
# while cap_right.isOpened() and cap_left.isOpened():
# while cap_right.isOpened():
# tf.Session.run()
model = tf.saved_model.load(MODEL_PATH)
infer = model.signatures["serving_default"]
print("infer: ", infer.structured_outputs)

while cap_left.isOpened():
    # cap_right.update()
    # cap_left.update()
    # retR, frame_right = cap_right.read()
    retL, frame_left = cap_left.read()
    # print("pre-undistort")
    frameL = frame_left.copy()
    # frameL = cv2.cvtColor(frameL, cv2.COLOR_BGR2RGB)
    # frameL = cv2.resize(frameL, (width, height))
    # input_data = np.expand_dims(frameL, axis=0)

    # input_data = (np.float32(input_data) - input_mean) / input_std

    # print("Keys: ", list(model.signatures.keys()))
    infer = model.signatures["serving_default"]
    output_details = infer.structured_outputs

    image_np = np.asarray(np.array(frameL))
    input_tensor = tf.convert_to_tensor(image_np)
    input_tensor = input_tensor[tf.newaxis, ...]
    input_tensor = input_tensor[:, :, :, :3]  # <= add this line
    BoxesTF = infer(input_tensor)["detection_boxes"][0].numpy()
    ClassesTF = infer(input_tensor)["detection_classes"][0].numpy()
    ScoresTF = infer(input_tensor)["detection_scores"][0].numpy()
    # print("Eval boxes: ", BoxesTF)
    # Undistort and rectify images
    # # Show the frames
    # cv2.imshow("frame right", frame_right)
    # cv2.imshow("frame left", frame_left)

    # We need grayscale for disparity map.
    # gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)
    # gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)

    # stereo = cv2.StereoBM(1, 16, 15)
    # disparity_image = stereo.compute(gray_left, gray_right)

    # disparity_image = depth_map(gray_left, gray_right)  # Get the disparity map

    # print("Boxes: ", BoxesTF)
    # print("Classes: ", ClassesTF)
    # print("Score: ", ScoresTF)
    # Perform the actual detection by running the model with the image as input
    # model.set_tensor(input_details[0]['index'], input_data)
    # interpreter.invoke()

    draw_detected_frame(frameL, 640, 480, BoxesTF, ClassesTF, ScoresTF)
    # Retrieve detection results
    # BoxesTF = model.get_tensor(output_details['detection_boxes'])  # Bounding box coordinates of detected objects
    # ClassesTF = model.get_tensor(output_details['detection_classes'])  # Class index of detected objects
    # ScoresTF = model.get_tensor(output_details['detection_scores'])  # Confidence of detected objects
    # num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and
    # not needed)

    # Show the images
    # cv2.imshow('left(R)', frame_left)
    # cv2.imshow('right(R)', frame_right)

    # cv2.imshow('Disparity', disparity_image)
    # Hit "q" to close the window
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# Release and destroy all windows before termination
# cap_right.stop()
cap_left.stop()

cv2.destroyAllWindows()
