#!python3
# Author: Theodor Giles
# Created: 7/14/21
# Last Edited 7/14/21
# Description:
# stereo depth map testing

import numpy as np
import cv2 as cv
# from matplotlib import pyplot as plt


imgL = cv.imread('tsukuba_l.png', 0)
imgR = cv.imread('tsukuba_r.png', 0)

stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
disparity = stereo.compute(imgL, imgR)

# plt.imshow(disparity,'gray')
# plt.show()