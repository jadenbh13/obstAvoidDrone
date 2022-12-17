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
from threading import Thread




dataSize = 8
divNum = 8

midNum = 8
x = np.arange(-0,dataSize,1)
y = np.arange(-0,dataSize,1)
block_size = 13
min_disp = 0
max_disp = 112
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

xCompensate = Value("f", 0)
zCompensate0 = Value("f", 0)
yCompensate = Value("f", 0)
zCompensate1 = Value("f", 0)

xValues = []




"""
  Args:
    X =  2D array of the Points on X-axis
    Y =  2D array of the Points on Y-axis
    r = goal size
    loc = goal location
  Return :
    delx and dely
  This function is to add the goal and its potential field on the graph.
  α = 50
"""
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

x = np.arange(-0,dataSize,1)
y = np.arange(-0,dataSize,1)
goal = random.sample(range(0, dataSize), 2)



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





"""def blur(disps):
    he, we = disps.shape
    stepx = int(we / divNum)
    stepy = int(he / divNum)
    yDim = int(he / stepy)
    xDim = int(we / stepx)
    newImageX = np.empty((yDim, xDim), int)
    newImageY = np.empty((xDim, yDim), int)
    hb = 0
    for yWind in range(0, int(he / stepy)):
        begy = stepy * yWind
        prevAv = 0
        for xWind in range(0, int(we / stepx)):
            begx = stepx * xWind
            colAvg = 0
            for yVT in range(begy, (begy + stepy)):
                for xVT in range(begx, (begx + stepx)):
                    if disps[yVT][xVT] <= 3:
                        disps[yVT][xVT] = 255
                    colAvg += disps[yVT][xVT]
            colAvg = colAvg / (stepx * stepy)

            hb += 1
            newImageX[yWind][xWind] = colAvg
            newImageY[xWind][yWind] = colAvg
            prevAv = 0
    return newImageX, newImageY"""



def blur(disps):
    print(type(disps))
    he, we = disps.shape
    stepx = int(we / divNum)
    stepy = int(he / divNum)
    yDim = int(he / stepy)
    xDim = int(we / stepx)
    newImageX = np.empty((yDim, xDim), int)
    newImageY = np.empty((xDim, yDim), int)
    hb = 0
    for yWind in range(0, int(he / stepy)):
        begy = stepy * yWind
        prevAv = 0
        for xWind in range(0, int(we / stepx)):
            begx = stepx * xWind
            colAvg = 0
            for yVT in range(begy, (begy + stepy)):
                for xVT in range(begx, (begx + stepx)):
                    if disps[yVT][xVT] <= 3:
                        disps[yVT][xVT] = 255
                    colAvg += disps[yVT][xVT]
            colAvg = colAvg / (stepx * stepy)

            hb += 1
            newImageX[yWind][xWind] = colAvg
            newImageY[xWind][yWind] = colAvg
            prevAv = 0
    print(newImageX)
    return newImageX, newImageY

"""def blur(disps):
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
    print(newImX)
    return newImX, newImY"""







goalX = dataSize * 0.5
goal = [goalX,dataSize]
fig, ax = plt.subplots(figsize = (10,10))
X, Y = np.meshgrid(x,y)

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
    """print(goal, goalX, xCompTot, yCompTot)"""
    #print(f"X: {xBias} {xCompTot}")
    #print(" ")
    """plt.show()"""
    #plt.close()
    #print(newImageXCurr[yPix])
    #print(xBias)
    #time.sleep(0.2)

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
        #mainMethod(newImageXs, yPix, radSize, addFactor, seek_points, originPoint, xCompensate, zCompensate0)
    """for process in procList:
        process.start()
    for process in procList:
        process.join()"""

    v0.value = xCompensate.value
    v1.value = zCompensate0.value
    #print(v0.value, v1.value)
    xCompensate.value = 0
    zCompensate0.value = 0
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

    disk = disps.copy()
    s = 7
    r=2
    originPoint = (dataSize * 0.5) + 0.0
    seek_points = np.array([[originPoint,0]])
    goalX = int(dataSize * 0.5)
    goal = [goalX,dataSize]
    stepx = int(we / divNum)
    stepy = int(he / divNum)
    yDim = int(he / stepy)
    xDim = int(we / stepx)
    blurred = blur(disps)
    #print(blurred)
    newImageX = blurred[0]
    newImageY = blurred[1]




            #print(f"{begx}  {begy}  {colAvg}")
    print("  ")
    #print(hb)
    #print(newImageY)
    radSize = 1.5

    goal = [goalX,dataSize]
    goal2 = [int(dataSize * 0.75),dataSize]
    goal3 = [int(dataSize * 0.25),dataSize]

    setPoin = 200


    addFactor = 1.5
    """try:
        for xPixels, yPixels in itertools.zip_longest(range(0, 10), range(0, 5)):
            print(xPixels, yPixels)
            print(len(newImageX), len(newImageY))
            #time.sleep(1)
    except Exception as e:
        print(e)"""
    #xVector0 = Value("f", 0)
    #xVector1 = Value("f", 0)
    #getLeanX(newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1)

    xVector0 = Value("f", 0)
    xVector1 = Value("f", 0)
    """process1 = Process(target=getLeanX, args=((newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1)))"""
    #xVector, totalXInt = getLeanX(newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1)
    tm1 = Thread(target=getLeanX, args=(newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1))
    #print(newImageX)

    yVector0 = Value("f", 0)
    yVector1 = Value("f", 0)
    """process2 = Process(target=getLeanY, args=((newImageY, radSize, addFactor, seek_points, originPoint, yVector0, yVector1)))"""
    #yVector, totalYInt = getLeanY(newImageY, radSize, addFactor, seek_points, originPoint, yVector0, yVector1)
    #tm2 = Thread(target=getLeanY, args=(newImageY, radSize, addFactor, seek_points, originPoint, yVector0, yVector1))
    tm1.start()
    #tm2.start()
    tm1.join()
    #tm2.join()
    plt.clf()

    """process1.start()
    process2.start()
    process1.join()
    process2.join()"""

    xVector = [float(xVector0.value), float(xVector1.value)]
    yVector = [float(yVector0.value), float(yVector1.value)]
    #xVector = [0, 0]
    #yVector = [0, 0]
    #xVector[0] = xVector[0] / len(newImageX)
    #totalXInt = totalXInt / len(newImageX)
    print(xVector, yVector)

    vectorMag = 40

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


    print(f"{xVector}  {xAngle}  {xDir}  {zDir}")
    print(f"{yVector}  {yAngle}  {yDir}  {zDir}")
    xVector = [0, 0]
    totalXInt = 0
    yVector = [0, 0]
    totalYInt = 0
    #cv2.imshow("pixel", disk)
    return xDir, yDir, zDir









frL = 0
frR = 0
qCounter = 0

#tello.send_rc_control(-10, 0, 0, 0)
#getImage = threading.Thread(target=thread_function, args=(1,))
#getImage.start()


executable = 0
cv2.setUseOptimized(True)
def obtainObst(frL, frR):
	#frL, frR = getPicNoMove()
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
	print("Got image")
	xVec, yVec, zVec = getDirection(disper)
	print(xVec, yVec, zVec)
	return disper



midNum = 60
widNum = 70
heiNum = 30
while True:
    fr0 = cv2.imread("left_img.png")
    fr1 = cv2.imread("right_img.png")
    he, we, ce = fr0.shape
    """frLs = fr0[0:(he-midNum), widNum:(we - widNum)]
    frRs = fr1[midNum:he, widNum:(we - widNum)]
    he, we, ce = frLs.shape"""
    #frL = frLs[heiNum:(he - heiNum), 0:we]
    #frR = frRs[heiNum:(he - heiNum), 0:we]
    frL = fr0
    frR = fr1
    k = cv2.waitKey(5)
    if k == ord('q'):
    	break
    elif k == ord('s'):

    	mainFrame = obtainObst(frL, frR)
    	cv2.imshow('disps', mainFrame)
    cv2.imshow('L', frL)
    cv2.imshow('R', frR)

cv2.destroyAllWindows()
