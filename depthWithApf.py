import matplotlib.pyplot as plt
from matplotlib import cm
import cv2
import numpy as np
import matplotlib
import math
import random
import time
from multiprocessing import Process
from multiprocessing import Value
from alternateApf import mainAvoid





img1_undistorted = cv2.imread("leftImTest2.png")
img2_undistorted = cv2.imread("rightImTest2.png")


block_size = 13
min_disp = 0
max_disp = 112
# Maximum disparity minus minimum disparity. The value is always greater than zero.
# In the current implementation, this parameter must be divisible by 16.
num_disp = max_disp - min_disp
# Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
# Normally, a value within the 5-15 range is good enough
uniquenessRatio = 10
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
disparity_SGBM = stereo.compute(img1_undistorted, img2_undistorted)

# Normalize the values to a range from 0..255 for a grayscale image
disparity_SGBM = cv2.normalize(disparity_SGBM, disparity_SGBM, alpha=255,
                              beta=0, norm_type=cv2.NORM_MINMAX)
disparity_SGBMs = np.uint8(disparity_SGBM)
shap = disparity_SGBMs.shape
disparity_SGBMs = disparity_SGBMs[0:shap[0], 160:shap[1]]
disparity_SGBMs = 255 - disparity_SGBMs
dispser = disparity_SGBMs





dataSize = 11
divNum = 8







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
Xs, Ys = np.meshgrid(x,y)




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









def mainMethod(newImageX, yPix, radSize, addFactor, seek_points, originPoint):
    goalX = dataSize * 0.5
    goal = [goalX,dataSize]
    #print(newImageX[yPix])
    #print(radSize, seek_points, originPoint, goal, goalX)

    #delx, dely =add_goal(X, Y,3, 8 , goal2)
    #delx, dely =add_goal(X, Y,3, 8 , goal3)
    #fig, ax = plt.subplots(figsize = (10,10))
    xBias = 0
    xVsList = []
    zVsList = []
    obstCordList = []
    prevAdd = 0
    radiusList = []
    #print('list: {0}'.format(newImageX[yPix]))
    rS = 0.0
    for xPix in range(0, len(newImageX[yPix])):
        zVal = newImageX[yPix][xPix] / (255 / dataSize) + (dataSize * 0.2)
        if zVal > dataSize:
            zVal = dataSize
        zRadius = zVal * (radSize / dataSize)
        zRadius = radSize - zRadius
        #print(zRadius, newImageX[yPix][xPix], xPix)
        if zRadius > (radSize * rS):
            if (xPix + 1) <= (divNum / 2):
                #print("left")
                xBias += (zRadius * addFactor)
            else:
                #print("right")
                xBias -= (zRadius * addFactor)
        #zRadius = zRadius ** 1.3Test2
        xValGraph = (xPix + 1) * (dataSize / len(newImageX[yPix]))
        #if zVal < (dataSize * 0.4):
        if zRadius > (radSize * rS):
            xVsList.append(xValGraph)
            zVsList.append(zVal)
            obstCordList.append([xValGraph, zVal])
            radiusList.append(zRadius)
    #print('obses: {0}'.format(obstCordList))
    xCompTot = 0
    yCompTot = 0

    if len(obstCordList) > 0:
        gv = mainAvoid(obstCordList, xBias, dataSize, radiusList, radSize, rS)
        xCompTot = gv[0]
        yCompTot = gv[1]
        #print(gv[0], gv[1])

    return xCompTot, yCompTot


def getLeanX(newImageList, radSize, addFactor, seek_points, originPoint, v0, v1):

    goalX = dataSize * 0.5
    goal = [goalX,dataSize]
    newImageX = newImageList
    xVector = [0, 0]
    totalXInt = 0
    procList = []
    xComp = 0
    yComp = 0
    for yPix in range(0, len(newImageX)):
        #proc = Process(target=mainMethod, args=(newImageX, yPix, radSize, addFactor, seek_points, originPoint, xCompensate, zCompensate))
        #procList.append(proc)
        xC, yC = mainMethod(newImageX, yPix, radSize, addFactor, seek_points, originPoint)
        xComp += xC
        yComp += yC
        #print(lb)

            #mainAvoid(lb, biased, dataSize)

    v0.value = xComp
    v1.value = yComp
    print(v0.value, v1.value)
    return xVector, totalXInt


def getLeanY(newImageList, radSize, addFactor, seek_points, originPoint, v0, v1):

    goalX = dataSize * 0.5
    goal = [goalX,dataSize]
    newImageX = newImageList
    yVector = [0, 0]
    totalYInt = 0
    yCompensate = Value("f", 0)
    zCompensate = Value("f", 0)
    for yPix in range(0, len(newImageX)):
        #proc = Process(target=mainMethod, args=(newImageX, yPix, radSize, addFactor, seek_points, originPoint, xCompensate, zCompensate))
        #procList.append(proc)
        lb, biased = mainMethod(newImageX, yPix, radSize, addFactor, seek_points, originPoint, yCompensate, zCompensate)
        #print(lb)

            #mainAvoid(lb, biased, dataSize)

    v0.value = yCompensate.value
    v1.value = zCompensate.value
    return yVector, totalYInt







def getDirection(disps):


    he, we = disps.shape

    disk = disps.copy()
    #s = 5
    #r=1
    originPoint = (dataSize * 0.5)
    print(originPoint)
    seek_points = np.array([[originPoint,0]])
    goalX = dataSize * 0.5
    goal = [goalX,dataSize]

    blurred = blur(disps)
    newImageX = blurred[0]
    newImageY = blurred[1]




            #print(f"{begx}  {begy}  {colAvg}")
    print("  ")
    #print(hb)
    #print(newImageY)
    radSize = 5.5

    goal = [goalX,dataSize]
    #goal2 = [int(dataSize * 0.75),dataSize]
    #goal3 = [int(dataSize * 0.25),dataSize]



    addFactor = 0.6

    #xVector0 = Value("f", 0)
    #xVector1 = Value("f", 0)
    #getLeanX(newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1)
    print("X: ")
    print(" ")
    xVector0 = Value("f", 0)
    xVector1 = Value("f", 0)
    #process1 = Process(target=getLeanX, args=((newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1)))
    xVector, totalXInt = getLeanX(newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1)
    #print("Y: ")
    #print(" ")
    yVector0 = Value("f", 0)
    yVector1 = Value("f", 0)
    #process2 = Process(target=getLeanY, args=((newImageY, radSize, addFactor, seek_points, originPoint, yVector0, yVector1)))
    #yVector, totalYInt = getLeanY(newImageY, radSize, addFactor, seek_points, originPoint, yVector0, yVector1)


    #process1.start()
    #process2.start()
    #process1.join()
    #process2.join()
    xDir = 0
    yDir = 0
    zDir = 0
    xVector = [float(xVector0.value), float(xVector1.value)]
    yVector = [float(yVector0.value), float(yVector1.value)]
    xAng = math.atan(xVector[0] / xVector[1]) * (180 / math.pi)
    xRadians = xAng * (math.pi / 180)
    xDir = int(math.sin(xRadians) * 40)
    zDir = int(math.cos(xRadians) * 40)
    print(xVector, xAng)
    print(xVector0.value, xVector1.value)
    print(f"X: {xDir}, Z: {zDir}")
    #cv2.imshow("pixel", disk)
    return xDir, yDir, zDir






"""
delx, dely =add_goal(X, Y, 3,  8, goal)

for yPix in range(0, len(disps)):
    if yPix == 280:
        for xPix in range(0, len(disps)):
            if disps[yPix][xPix] < 200.0:
                zVal = int(disps[yPix][xPix] / 2.55)
                xValGraph = int(xPix / 4.8)
                delx, dely, loc, r = add_obstacle(X,Y, delx,dely,goal, xValGraph, zVal)
                plot_graph(X, Y, delx, dely , 'Obstacle',fig, ax, loc, r , 'm')
                #print(f"{xPix}   {yPix}   {disps[yPix][xPix]}  {zVal}")
"""


disparity_SGBM = stereo.compute(img1_undistorted, img2_undistorted)

# Normalize the values to a range from 0..255 for a grayscale image
disparity_SGBM = cv2.normalize(disparity_SGBM, disparity_SGBM, alpha=255,
                              beta=0, norm_type=cv2.NORM_MINMAX)
disparity_SGBMs = np.uint8(disparity_SGBM)
shap = disparity_SGBMs.shape
disparity_SGBMs = disparity_SGBMs[0:shap[0], 160:shap[1]]
disparity_SGBMs = 255 - disparity_SGBMs
dispser = disparity_SGBMs
tm = time.time()
xD, yD, zD = getDirection(dispser)
#print(f"Total vector:  {xD}  {yD}  {zD}")
tm2 = time.time()
print(tm2 - tm)
cv2.imshow('gray', dispser)
cv2.waitKey()
