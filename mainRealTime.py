import matplotlib.pyplot as plt
"""from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm"""
import cv2
import numpy as np
"""from PIL import Image"""
import math
"""from numpy.lib.stride_tricks import as_strided
from dataclasses import dataclass"""
import matplotlib
#from skimage.feature import blob_dog, blob_log, blob_doh
import math
"""from scipy.signal import argrelextrema, find_peaks"""
import random
import time
"""import itertools"""
from multiprocessing import Process
from multiprocessing import Value
import threading







dataSize = 9
divNum = 9

midNum = 8
x = np.arange(-0,dataSize,1)
y = np.arange(-0,dataSize,1)
block_size = 11
min_disp = 0
max_disp = 32
# Maximum disparity minus minimum disparity. The value is always greater than zero.
# In the current implementation, this parameter must be divisible by 16.
num_disp = max_disp - min_disp
# Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
# Normally, a value within the 5-15 range is good enough
uniquenessRatio = 7
# Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
# Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
speckleWindowSize = 200
# Maximum disparity variation within each connected component.
# If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16.
# Normally, 1 or 2 is good enough.
speckleRange = 2
disp12MaxDiff = 0

stereo = cv2.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=block_size,
    uniquenessRatio=uniquenessRatio,
    speckleWindowSize=speckleWindowSize,
    speckleRange=speckleRange,
    disp12MaxDiff=disp12MaxDiff,
    P1=8 * 1 * block_size * block_size,
    P2=32 * 1 * block_size * block_size,
)











frL = 0
frR = 0
qCounter = 0
"""print("Tello initialized")
print("Taking off in: ")
for i in range(0, 3):
    h = 3 - i
    print(h)
    time.sleep(1)
print("Taking off...")
tello.takeoff()
bats = int(tello.get_battery())
print(bats)"""
#tello.send_rc_control(-10, 0, 0, 0)
#getImage = threading.Thread(target=thread_function, args=(1,))
#getImage.start()
cv2.setUseOptimized(True)

executable = 0

leftFrame = cv2.imread("currentLeft.png")
rightFrame = cv2.imread("currentRight.png")

def obtainObst():
	global leftFrame
	global rightFrame
	while True:
		#frL, frR = getPicNoMove()
		frL = leftFrame
		frR = rightFrame
		img1_undistorted = frL
		img2_undistorted = frR
		#cv2.imwrite("droneLeft.png", frL)
		#cv2.imwrite("droneRight.png", frR)

		disparity_SGBM = stereo.compute(img1_undistorted, img2_undistorted)

		# Normalize the values to a range from 0..255 for a grayscale image
		disparity_SGBM = cv2.normalize(disparity_SGBM, disparity_SGBM, alpha=255,
									  beta=0, norm_type=cv2.NORM_MINMAX)
		disparity_SGBMs = np.uint8(disparity_SGBM)
		shap = disparity_SGBMs.shape
		disparity_SGBMs = disparity_SGBMs[0:shap[0], 70:shap[1]]
		disparity_SGBMs = disparity_SGBMs
		disper = 255 - disparity_SGBMs
		#print("Got image")
		#cv2.imshow("L", frL)
		#cv2.imshow("R", frR)
		cv2.imshow("Disp", disper)
		k = cv2.waitKey(5)
		if k == ord('q'):
			break
		"""xVec, yVec, zVec = getDirection(disper)"""
		"""print(xVec, yVec, zVec)"""
	cv2.destroyAllWindows()

def foreground():
	global leftFrame
	global rightFrame
	video_capture_0 = cv2.VideoCapture('/dev/video0')
	video_capture_1 = cv2.VideoCapture('/dev/video2')
	midNum = 60
	widNum = 60
	heiNum = 30
	while True:
		ret0, fr0 = video_capture_0.read()
		ret1, fr1 = video_capture_1.read()
		he, we, ce = fr0.shape
		frLs = fr0[0:(he-midNum), widNum:(we - widNum)]
		frRs = fr1[midNum:he, widNum:(we - widNum)]
		he, we, ce = frLs.shape
		frL = frLs[heiNum:(he - heiNum), 0:we]
		frR = frRs[heiNum:(he - heiNum), 0:we]
		leftFrame = frL
		rightFrame = frR

b = threading.Thread(target=obtainObst)
f = threading.Thread(target=foreground)

b.start()
f.start()
