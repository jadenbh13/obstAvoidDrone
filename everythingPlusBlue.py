import matplotlib.pyplot as plt
plt.switch_backend('agg')

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
#from multiprocessing import Process
from multiprocessing import Value
import threading
from threading import Thread
import bluetooth






dataSize = 8
divNum = 6

midNum = 8
x = np.arange(-0,dataSize,1)
y = np.arange(-0,dataSize,1)
xVector0 = Value("f", 0)
xVector1 = Value("f", 0)
yVector0 = Value("f", 0)
yVector1 = Value("f", 0)
block_size = 13
min_disp = 0
max_disp = 16
# Maximum disparity minus minimum disparity. The value is always greater than zero.
# In the current implementation, this parameter must be divisible by 16.
num_disp = max_disp - min_disp
# Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
# Normally, a value within the 5-15 range is good enough
uniquenessRatio = 5
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

xCompensate = Value("f", 0)
zCompensate0 = Value("f", 0)
yCompensate = Value("f", 0)
zCompensate1 = Value("f", 0)
fig, ax = plt.subplots(figsize = (10,10))
X, Y = np.meshgrid(x,y)
originPoint = (dataSize * 0.5) + 0.0
seek_points = np.array([[originPoint,0]])
cv2.setUseOptimized(True)
vectorMag = 40

server_sock = bluetooth.BluetoothSocket(bluetooth.L2CAP)
port = 0x1001
server_sock.bind(("", port))
server_sock.listen(1)














def add_goal (X, Y,s, r, loc):

    delx = np.zeros_like(X)
    dely = np.zeros_like(Y)
    for i in range(len(x)):
        for j in range(len(y)):

            d= np.sqrt((loc[0]-X[i][j])**2 + (loc[1]-Y[i][j])**2)
            #print(f"{i} and {j}")
            theta = np.arctan2(loc[1]-Y[i][j], loc[0] - X[i][j])
            if d< r:
                delx[i][j] = 0
                dely[i][j] =0
            elif d>r+s:
                delx[i][j] = dataSize* s *np.cos(theta)
                dely[i][j] = dataSize * s *np.sin(theta)
            else:
                delx[i][j] = dataSize * (d-r) *np.cos(theta)
                dely[i][j] = dataSize * (d-r) *np.sin(theta)
    return delx, dely

#x = np.arange(-0,dataSize,1)
#y = np.arange(-0,dataSize,1)
#goal = random.sample(range(0, dataSize), 2)



def plot_graph(X, Y, delx, dely,obj, fig, ax, loc,r,color,start_goal=np.array([[0,0]])  ):

    ax.quiver(X, Y, delx, dely)
    ax.add_patch(plt.Circle(loc, r, color=color))
    ax.annotate(obj, xy=loc, fontsize=10, ha="center")
    return ax



def add_obstacle(X, Y , delx, dely, goal, xLoc, yLoc, rads):
    s = (rads + 1) * 1.8

    # generating obstacle with random sizes
    r = rads
    ghy = 0
    # generating random location of the obstacle
    obstacle0 = xLoc
    obstacle1 = yLoc
    obstacle = (xLoc, yLoc)
    for i in range(len(x)):
        for j in range(len(y)):

            d_goal = np.sqrt((goal[0]-X[i][j])**2 + ((goal[1]-Y[i][j]))**2)
            d_obstacle = np.sqrt((obstacle[0]-X[i][j])**2 + (obstacle[1]-Y[i][j])**2)
            #print(f"{i} and {j}")
            theta_goal= np.arctan2(goal[1] - Y[i][j], goal[0]  - X[i][j])
            theta_obstacle = np.arctan2(obstacle[1] - Y[i][j], obstacle[0]  - X[i][j])
            if d_obstacle < r:
                delx[i][j] = -1*np.sign(np.cos(theta_obstacle))*5 +0
                dely[i][j] = -1*np.sign(np.cos(theta_obstacle))*5  +0
            elif d_obstacle>r+s:
                delx[i][j] += 0 -(dataSize * s *np.cos(theta_goal))
                dely[i][j] += 0 - (dataSize * s *np.sin(theta_goal))
            elif d_obstacle<r+s :
                delx[i][j] += (dataSize * -3) *(s+r-d_obstacle)* np.cos(theta_obstacle)
                dely[i][j] += (dataSize * -3) * (s+r-d_obstacle)*  np.sin(theta_obstacle)
            if d_goal <r+s:
                if delx[i][j] != 0:

                    delx[i][j]  += (dataSize * (d_goal-r) *np.cos(theta_goal))
                    dely[i][j]  += (dataSize * (d_goal-r) *np.sin(theta_goal))
                else:

                    delx[i][j]  = (dataSize * (d_goal-r) *np.cos(theta_goal))
                    dely[i][j]  = (dataSize * (d_goal-r) *np.sin(theta_goal))

            if d_goal>r+s:
                if delx[i][j] != 0:
                    delx[i][j] += dataSize* s *np.cos(theta_goal)
                    dely[i][j] += dataSize* s *np.sin(theta_goal)
                else:

                    delx[i][j] = dataSize* s *np.cos(theta_goal)
                    dely[i][j] = dataSize* s *np.sin(theta_goal)
            if d_goal<r:
                delx[i][j] = 0
                dely[i][j] = 0

    return delx, dely, obstacle, r





def blur(disps):
    he, we = disps.shape
    #print(he, we)
    yDim = int(he / divNum)
    xDim = int(we / divNum)
    newImX = np.empty((divNum, divNum), int)
    newImY = np.empty((divNum, divNum), int)
    for ye in range(0, divNum):
        yD = yDim * ye
        for xe in range(0, divNum):
            xD = xe * xDim
            #print(xD, yD)
            arrAlt = disps[yD:(yD + yDim), xD:(xD + xDim)]
            currMean = int(np.mean(arrAlt))
            newImX[ye][xe] = currMean
            newImY[xe][ye] = currMean
    #print(newImX)
    return newImX, newImY



def mainMethod(newImageXCurr, yPix, radSize, addFactor, seek_points, originPoint, x0, z0):
    goalX = dataSize * 0.5
    goal = [goalX,dataSize]
    #print(newImageX[yPix])
    #print(radSize, seek_points, originPoint, goal, goalX)
    #fig, ax = plt.subplots(figsize = (10,10))
    #delx, dely =add_goal(X, Y,3, 8 , goal2)
    #delx, dely =add_goal(X, Y,3, 8 , goal3)
    xBias = 0
    xVsList = []
    zVsList = []
    prevAdd = 0
    for xPix in range(0, len(newImageXCurr[yPix])):
        zVal = int(newImageXCurr[yPix][xPix] / (255 / dataSize)) + (dataSize * 0.25)
        if zVal > dataSize:
            zVal = dataSize
        #print(xErr, zVal)
        zRadius = zVal * (radSize / dataSize)
        zRadius = radSize - zRadius
        if xPix < (dataSize * 0.5):
            xBias += (zRadius * addFactor)

        elif xPix >= (dataSize * 0.5):
            xBias -= (zRadius * addFactor)

        #zRadius = zRadius ** 1.3
        xValGraph = int(xPix * (dataSize / len(newImageXCurr[yPix])))
        xVsList.append(xValGraph)
        zVsList.append(zVal)
        #print(f"{zVal}  {xValGraph}  {xPix}  {yPix}  {zRadius}")
    goal = [(goalX + xBias), dataSize]
    delx, dely =add_goal(X.copy(), Y.copy(),2, 2 , goal)
    #obsListLocs = []
    #obsListRads = []
    #print(' ')
    for hg in range(0, len(xVsList)):
        zRadius = zVsList[hg] * (radSize / dataSize)
        zRadius = radSize - zRadius
        #print(zRadius)
        #obsListLocs.append((xVsList[hg], zVsList[hg]))
        #obsListRads.append(zRadius)
        delx, dely, loc, r = add_obstacle(X,Y, delx,dely,goal, xVsList[hg], zVsList[hg], zRadius)
        """plot_graph(X, Y, delx, dely , 'Obstacle',fig, ax, loc, r , 'm')"""

    strm = ax.streamplot(X,Y,delx,dely, start_points=seek_points,linewidth=3, cmap='autu')
    segs = strm.lines.get_segments()
    num_pts = len(segs)
    flow_line = np.full((num_pts, 2), np.nan)
    for i in range(num_pts):
        flow_line[i,:] = segs[i][0,:]
    prevXV = 0
    prevYV = 0
    prevDeriv = 0
    xIntegral = 0
    xCompTot = 0
    yCompTot = 0
    for o in flow_line:
        if o[1] < (dataSize * 0.6):
            xV = (round(o[0], 2) - originPoint)
            yV = (round(o[1], 2))
            xCompTot += xV
            yCompTot += (yV - prevYV)
            #print(yCompTot)
            #currXInt = (((xV + prevXV) * 0.5) * (yV - prevYV))
            #xIntegral += currXInt
            prevXV = xV
            prevYV = yV


    x0.value = x0.value + xCompTot
    z0.value = z0.value + yCompTot
    return x0.value, z0.value


def getLeanX(newImageList, radSize, addFactor, seek_points, originPoint, v0, v1):
    newImageXs = newImageList
    xVector = [0, 0]
    totalXInt = 0
    procList = []
    for yPix in range(0, len(newImageXs)):
        #proc = Process(target=mainMethod, args=(newImageX, yPix, radSize, addFactor, seek_points, originPoint, xCompensate, zCompensate))
        #procList.append(proc)
        t1 = Thread(target=mainMethod, args=(newImageXs, yPix, radSize, addFactor, seek_points, originPoint, xCompensate, zCompensate0))
        t1.start()
        t1.join()
        #mainMethod(newImageXs, yPix, radSize, addFactor, seek_points, originPoint, xCompensate, zCompensate0)


    v0.value = xCompensate.value
    v1.value = zCompensate0.value
    #print(v0.value, v1.value)
    xCompensate.value = 0
    zCompensate0.value = 0
    plt.clf()
    return xVector, totalXInt


def getLeanY(newImageList, radSize, addFactor, seek_points, originPoint, v0, v1):
    goalX = int(dataSize * 0.5)
    goal = [goalX,dataSize]
    newImageYs = newImageList
    yVector = [0, 0]
    totalYInt = 0
    procList = []
    for yPix in range(0, len(newImageYs)):
        #proc = Process(target=mainMethod, args=(newImageY, yPix, radSize, addFactor, seek_points, originPoint, yCompensate, zCompensate))
        #procList.append(proc)
        t2 = Thread(target=mainMethod, args=(newImageYs, yPix, radSize, addFactor, seek_points, originPoint, yCompensate, zCompensate1))
        t2.start()
        #mainMethod(newImageYs, yPix, radSize, addFactor, seek_points, originPoint, yCompensate, zCompensate1)
    """for process in procList:
        process.start()
    for process in procList:
        process.join()"""

    v0.value = yCompensate.value
    v1.value = zCompensate1.value
    yCompensate.value = 0
    zCompensate1.value = 0
    return yVector, totalYInt







def getDirection(disps):

    he, we = disps.shape
    s = 7
    r=2

    goalX = dataSize * 0.5
    goal = [goalX,dataSize]
    stepx = int(we / divNum)
    stepy = int(he / divNum)
    yDim = int(he / stepy)
    xDim = int(we / stepx)
    blurred = blur(disps)
    #print(blurred)
    newImageX = blurred[0]
    newImageY = blurred[1]

    #print("  ")
    #print(hb)
    #print(newImageY)
    radSize = 1.5



    setPoin = 200

    addFactor = 1.5

    #xVector0 = Value("f", 0)
    #xVector1 = Value("f", 0)
    getLeanX(newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1)

    #tm1 = Thread(target=getLeanX, args=(newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1))

    #tm2 = Thread(target=getLeanY, args=(newImageY, radSize, addFactor, seek_points, originPoint, yVector0, yVector1))
    #tm1.start()
    #tm2.start()
    #tm1.join()
    #tm2.join()
    #plt.clf()


    xVector = [xVector0.value, xVector1.value]
    yVector = [yVector0.value, yVector1.value]
    #xVector = [0, 0]
    #yVector = [0, 0]
    #xVector[0] = xVector[0] / len(newImageX)
    #totalXInt = totalXInt / len(newImageX)
    #print(xVector, yVector)


    xVector[0] = round(xVector[0], 2)
    xVector[1] = round(xVector[1], 2)
    yVector[0] = round(yVector[0], 2)
    yVector[1] = round(yVector[1], 2)
    if xVector[0] == 0:
        xVector[0] = 0.1
    if yVector[0] == 0:
        yVector[0] = 0.1

    xAngle = math.atan(xVector[1] / xVector[0]) * (180 / math.pi)
    yAngle = math.atan(yVector[1] / yVector[0]) * (180 / math.pi)
    xMag = math.sqrt((xVector[0] ** 2) + (xVector[1] ** 2))
    yMag = math.sqrt((yVector[0] ** 2) + (yVector[1] ** 2))
    xRadians = xAngle * (math.pi / 180)
    yRadians = yAngle * (math.pi / 180)
    xDir = 0
    yDir = 0
    if xAngle < 0:
        xDir = (-1) * math.cos(abs(xRadians))
    elif xAngle > 0:
        xDir = math.cos(abs(xRadians))

    if yAngle < 0:
        yDir = (-1) * math.cos(abs(yRadians))
    elif yAngle > 0:
        yDir = math.cos(abs(yRadians))

    zDir = (1/2) * ((math.sin(abs(xRadians)) ** 1) + (math.sin(abs(yRadians)) ** 1))
    xDir = int(vectorMag * xDir)
    yDir = int(vectorMag * yDir)
    zDir = int(vectorMag * zDir)


    #print(f"{xVector}  {xAngle}  {xDir}  {zDir}")
    #print(f"{yVector}  {yAngle}  {yDir}  {zDir}")
    xVector = [0, 0]
    totalXInt = 0
    yVector = [0, 0]
    totalYInt = 0
    xVector0.value = 0
    xVector1.value = 0
    yVector0.value = 0
    yVector1.value = 0
    #cv2.imshow("pixel", disk)
    #plt.clf()
    return xDir, yDir, zDir



















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


executable = 0

leftFrame = cv2.imread("currentLeft.png")
rightFrame = cv2.imread("currentRight.png")

def obtainObst(client_sock, h):
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

		disparity_SGBM = stereo.compute(img2_undistorted, img1_undistorted)

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
		xVec, yVec, zVec = getDirection(disper)
		print(xVec, yVec, zVec)
		client_sock.send(str(xVec).encode())
		client_sock.send(str(zVec).encode())
	print("Closing everything...")
	cv2.destroyAllWindows()
	client_sock.close()
	server_sock.close()
	print("Everything closed")

def foreground():
	global leftFrame
	global rightFrame
	video_capture_0 = cv2.VideoCapture('/dev/video0')
	video_capture_1 = cv2.VideoCapture('/dev/video2')
	midNum = 35
	widNum = 65
	heiNum = 50
	while True:
		ret0, fr0 = video_capture_0.read()
		ret1, fr1 = video_capture_1.read()
		he, we, ce = fr0.shape
		frLs = fr0[midNum:he, widNum:(we - widNum)]
		frRs = fr1[0:(he-midNum), widNum:(we - widNum)]

		he1, we1, ce1 = frLs.shape
		frL = frLs[heiNum:(he1 - heiNum), 0:we1]
		frR = frRs[heiNum:(he1 - heiNum), 0:we1]
		leftFrame = frL
		rightFrame = frR
client_sock, address = server_sock.accept()
print("Accepted connection from", address)
b = threading.Thread(target=obtainObst, args=(client_sock, 1))
f = threading.Thread(target=foreground)
b.start()
f.start()
