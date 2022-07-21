import numpy as np
import cv2

# from matplotlib import pyplot as plt
CamL = cv2.VideoCapture(0)
CamR = cv2.VideoCapture(1)
cv2.namedWindow('disp', cv2.WINDOW_NORMAL)
# Reading the mapping values for stereo image rectification
cv_file = cv2.FileStorage("stereo_rectify_maps.xml", cv2.FILE_STORAGE_READ)
Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
cv_file.release()

while True:
    retL, imgL = CamL.read()
    retR, imgR = CamR.read()
    if retL and retR:
        print("in return func")
        # imgL = np.asarray(imgL,dtype=np.float32)
        # imgR = np.asarray(imgR,dtype=np.float32)
        # rgbL = cv2.cvtColor(imgL, cv2.COLOR_GRAY2RGB)
        # rgbR = cv2.cvtColor(imgR, cv2.COLOR_GRAY2RGB)
        grayL = cv2.cvtColor(imgL, cv2.COLOR_RGB2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_RGB2GRAY)
        # Applying stereo image rectification on the left image
        Left_nice = cv2.remap(grayL,
                              Left_Stereo_Map_x,
                              Left_Stereo_Map_y,
                              cv2.INTER_LANCZOS4,
                              cv2.BORDER_CONSTANT,
                              0)

        # Applying stereo image rectification on the right image
        Right_nice = cv2.remap(grayR,
                               Right_Stereo_Map_x,
                               Right_Stereo_Map_y,
                               cv2.INTER_LANCZOS4,
                               cv2.BORDER_CONSTANT,
                               0)

        # Converting to float32
        disparity = disparity.astype(np.float32)

        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(Left_nice, Right_nice)

    cv2.imshow("disp", disparity)
