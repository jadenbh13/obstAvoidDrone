import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import cv2
import numpy as np
from PIL import Image
import math
from numpy.lib.stride_tricks import as_strided
from dataclasses import dataclass
import matplotlib
#from skimage.feature import blob_dog, blob_log, blob_doh
import math
from scipy.signal import argrelextrema, find_peaks
import random
import time
import itertools
from multiprocessing import Process
from multiprocessing import Value





img1_undistorted = cv2.imread("left_img.png")
img2_undistorted = cv2.imread("right_img.png")


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





dataSize = 10
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



def plot_graph(X, Y, delx, dely,obj, fig, ax, loc,r,color,start_goal=np.array([[0,0]])  ):

    ax.quiver(X, Y, delx, dely)
    ax.add_patch(plt.Circle(loc, r, color=color))
    ax.annotate(obj, xy=loc, fontsize=10, ha="center")
    return ax



def add_obstacle(X, Y , delx, dely, goal, xLoc, yLoc, rads):
    s = (rads + 1) * 1.5

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




def add_obstacles(X, Y , delx, dely, goal, locList, radList):
    ghy = 0
    # generating random location of the obstacle
    #obstacle0 = xLoc
    #obstacle1 = yLoc
    #obstacle = (xLoc, yLoc)
    print(locList, radList)
    for i in range(len(x)):
        for j in range(len(y)):

            d_goal = np.sqrt((goal[0]-X[i][j])**2 + ((goal[1]-Y[i][j]))**2)
            #d_obstacle = np.sqrt((obstacle[0]-X[i][j])**2 + (obstacle[1]-Y[i][j])**2)
            #print(f"{i} and {j}")
            theta_goal= np.arctan2(goal[1] - Y[i][j], goal[0]  - X[i][j])
            #theta_obstacle = np.arctan2(obstacle[1] - Y[i][j], obstacle[0]  - X[i][j])
            dObsList = []
            thetaObsList = []
            for ind in range(0, len(locList)):
                d_obs = np.sqrt((locList[ind][0]-X[i][j])**2 + (locList[ind][1]-Y[i][j])**2)
                theta_obs = np.arctan2(locList[ind][1] - Y[i][j], locList[ind][0]  - X[i][j])
                dObsList.append(d_obs)
                thetaObsList.append(theta_obs)

            for jInd in range(0, len(dObsList)):
                s = (radList[jInd] + 1.0) * 2.0
                r = radList[jInd]
                d_obstacle = dObsList[jInd]
                theta_obstacle = thetaObsList[jInd]


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

    return delx, dely



def blur(disps):
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
            """for yVT in range(begy, (begy + stepy)):
                for xVT in range(begx, (begx + stepx)):
                    disk[yVT][xVT] = int(colAvg)"""
            hb += 1
            newImageX[yWind][xWind] = colAvg
            newImageY[xWind][yWind] = colAvg
            prevAv = 0
    return newImageX, newImageY






goalX = int(dataSize * 0.5)
goal = [goalX,dataSize]


def mainMethod(newImageX, yPix, radSize, addFactor, seek_points, originPoint, x0, z0):
    goalX = int(dataSize * 0.5)
    goal = [goalX,dataSize]
    #print(newImageX[yPix])
    #print(radSize, seek_points, originPoint, goal, goalX)

    #delx, dely =add_goal(X, Y,3, 8 , goal2)
    #delx, dely =add_goal(X, Y,3, 8 , goal3)
    fig, ax = plt.subplots(figsize = (10,10))
    xBias = 0
    xVsList = []
    zVsList = []
    prevAdd = 0
    for xPix in range(0, len(newImageX[yPix])):
        zVal = int(newImageX[yPix][xPix] / (255 / dataSize)) + (dataSize * 0.25)
        if zVal > dataSize:
            zVal = dataSize
        zRadius = zVal * (radSize / dataSize)
        zRadius = radSize - zRadius
        xErr = (len(newImageX[yPix]) * 0.5) - xPix
        xAdd = zRadius * 1.0
        if xErr < 0.0:
            xAdd *= (-1.0)
        elif xErr > 0.0:
            xAdd *= (1.0)
        xBias += ((xAdd + prevAdd) * 0.5) * addFactor
        prevAdd = xAdd
        #zRadius = zRadius ** 1.3
        xValGraph = int(xPix * (dataSize / len(newImageX[yPix])))
        xVsList.append(xValGraph)
        zVsList.append(zVal)
        #print(f"{zVal}  {xValGraph}  {xPix}  {yPix}  {zRadius}")
    goal = [(goalX + xBias), dataSize]
    X, Y = np.meshgrid(x,y)
    delx, dely =add_goal(X, Y,2, 2 , goal)
    obsListLocs = []
    obsListRads = []
    for hg in range(0, len(xVsList)):
        zRadius = zVsList[hg] * (radSize / dataSize)
        zRadius = radSize - zRadius
        obsListLocs.append((xVsList[hg], zVsList[hg]))
        obsListRads.append(zRadius)
        delx, dely, loc, r = add_obstacle(X,Y, delx,dely,goal, xVsList[hg], zVsList[hg], zRadius)
        plot_graph(X, Y, delx, dely , 'Obstacle',fig, ax, loc, r , 'm')

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
        if o[1] < (dataSize * 0.7):
            xV = (round(o[0], 2) - originPoint)
            yV = (round(o[1], 2))
            xCompTot += xV
            yCompTot += (yV - prevYV)
            #print(yCompTot)
            currXInt = (((xV + prevXV) * 0.5) * (yV - prevYV))
            xIntegral += currXInt
            prevXV = xV
            prevYV = yV
    print(radSize, seek_points, originPoint, goal, goalX)
    #print(f"X: {xBias} {xCompTot}")
    #print(" ")
    plt.show()
    plt.close()
    time.sleep(0.2)
    x0.value = x0.value + xCompTot
    z0.value = z0.value + yCompTot


def getLeanX(newImageList, radSize, addFactor, seek_points, originPoint, v0, v1):

    goalX = int(dataSize * 0.5)
    goal = [goalX,dataSize]
    newImageX = newImageList
    xVector = [0, 0]
    totalXInt = 0
    procList = []
    xCompensate = Value("f", 0)
    zCompensate = Value("f", 0)
    for yPix in range(0, len(newImageX)):
        proc = Process(target=mainMethod, args=(newImageX, yPix, radSize, addFactor, seek_points, originPoint, xCompensate, zCompensate))
        procList.append(proc)
        #mainMethod(newImageX, yPix, radSize, addFactor, seek_points, originPoint)
    for process in procList:
        process.start()
    for process in procList:
        process.join()
        """X, Y = np.meshgrid(x,y)
        delx, dely =add_goal(X, Y,2, 2 , goal)
        #delx, dely =add_goal(X, Y,3, 8 , goal2)
        #delx, dely =add_goal(X, Y,3, 8 , goal3)
        fig, ax = plt.subplots(figsize = (10,10))
        xBias = 0
        xVsList = []
        zVsList = []
        prevAdd = 0
        for xPix in range(0, len(newImageX[yPix])):
            zVal = int(newImageX[yPix][xPix] / (255 / dataSize)) + (dataSize * 0.25)
            if zVal > dataSize:
                zVal = dataSize
            zRadius = zVal * (radSize / dataSize)
            zRadius = radSize - zRadius
            xErr = (len(newImageX[yPix]) * 0.5) - xPix
            xAdd = zRadius * 1.0
            if xErr < 0.0:
                xAdd *= (-1.0)
            elif xErr > 0.0:
                xAdd *= (1.0)
            xBias += ((xAdd + prevAdd) * 0.5) * addFactor
            prevAdd = xAdd
            #zRadius = zRadius ** 1.3
            xValGraph = int(xPix * (dataSize / len(newImageX[yPix])))
            xVsList.append(xValGraph)
            zVsList.append(zVal)
            #print(f"{zVal}  {xValGraph}  {xPix}  {yPix}  {zRadius}")
        goal = [(goalX + xBias), dataSize]
        obsListLocs = []
        obsListRads = []
        for hg in range(0, len(xVsList)):
            zRadius = zVsList[hg] * (radSize / dataSize)
            zRadius = radSize - zRadius
            obsListLocs.append((xVsList[hg], zVsList[hg]))
            obsListRads.append(zRadius)
            delx, dely, loc, r = add_obstacle(X,Y, delx,dely,goal, xVsList[hg], zVsList[hg], zRadius)
            #plot_graph(X, Y, delx, dely , 'Obstacle',fig, ax, loc, r , 'm')

        #delx, dely = add_obstacles(X,Y, delx,dely,goal, obsListLocs, obsListRads)
        #print(obsListLocs)
        #print(obsListRads)
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
            if o[1] < (dataSize * 0.7):
                xV = (round(o[0], 2) - originPoint)
                yV = (round(o[1], 2))
                xCompTot += xV
                yCompTot += (yV - prevYV)
                #print(yCompTot)
                currXInt = (((xV + prevXV) * 0.5) * (yV - prevYV))
                xIntegral += currXInt
                prevXV = xV
                prevYV = yV
        xVector[0] += xCompTot
        xVector[1] += yCompTot
        print(f"X: {xBias} {xVector}")
        print(" ")
        #plt.show()
        plt.close()
        time.sleep(0.2)"""
    v0.value = xCompensate.value
    v1.value = zCompensate.value
    print(v0.value, v1.value)
    return xVector, totalXInt


def getLeanY(newImageList, radSize, addFactor, seek_points, originPoint, v0, v1):

    goalX = int(dataSize * 0.5)
    goal = [goalX,dataSize]
    newImageX = newImageList
    yVector = [0, 0]
    totalYInt = 0
    for yPix in range(0, len(newImageX)):

        X, Y = np.meshgrid(x,y)
        delx, dely =add_goal(X, Y,2, 2 , goal)
        #delx, dely =add_goal(X, Y,3, 8 , goal2)
        #delx, dely =add_goal(X, Y,3, 8 , goal3)
        fig, ax = plt.subplots(figsize = (10,10))
        xBias = 0
        xVsList = []
        zVsList = []
        prevAdd = 0
        for xPix in range(0, len(newImageX[yPix])):
            zVal = int(newImageX[yPix][xPix] / (255 / dataSize)) + (dataSize * 0.25)
            if zVal > dataSize:
                zVal = dataSize
            zRadius = zVal * (radSize / dataSize)
            zRadius = radSize - zRadius
            xErr = (len(newImageX[yPix]) * 0.5) - xPix
            xAdd = zRadius * 1.0
            if xErr < 0.0:
                xAdd *= (-1.0)
            elif xErr > 0.0:
                xAdd *= (1.0)
            xBias += ((xAdd + prevAdd) * 0.5) * addFactor
            prevAdd = xAdd
            #zRadius = zRadius ** 1.3
            xValGraph = int(xPix * (dataSize / len(newImageX[yPix])))
            xVsList.append(xValGraph)
            zVsList.append(zVal)
            #print(f"{zVal}  {xValGraph}  {xPix}  {yPix}  {zRadius}")
        goal = [(goalX + xBias), dataSize]
        obsListLocs = []
        obsListRads = []
        for hg in range(0, len(xVsList)):
            zRadius = zVsList[hg] * (radSize / dataSize)
            zRadius = radSize - zRadius
            obsListLocs.append((xVsList[hg], zVsList[hg]))
            obsListRads.append(zRadius)
            delx, dely, loc, r = add_obstacle(X,Y, delx,dely,goal, xVsList[hg], zVsList[hg], zRadius)
            plot_graph(X, Y, delx, dely , 'Obstacle',fig, ax, loc, r , 'm')
        #delx, dely = add_obstacles(X,Y, delx,dely,goal, obsListLocs, obsListRads)
        #print(obsListLocs)
        #print(obsListRads)
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
            if o[1] < (dataSize * 0.7):
                xV = (round(o[0], 2) - originPoint)
                yV = (round(o[1], 2))
                xCompTot += xV
                yCompTot += (yV - prevYV)
                #print(yCompTot)
                currXInt = (((xV + prevXV) * 0.5) * (yV - prevYV))
                xIntegral += currXInt
                prevXV = xV
                prevYV = yV
        yVector[0] += xCompTot
        yVector[1] += yCompTot
        print(f"Y: {xBias} {yVector}")
        print(" ")
        plt.show()
        plt.close()
        time.sleep(0.2)
    v0.value = yVector[0]
    v1.value = yVector[1]
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
    blurred = blur(disps);
    newImageX = blurred[0]
    newImageY = blurred[1]




            #print(f"{begx}  {begy}  {colAvg}")
    print("  ")
    #print(hb)
    #print(newImageY)
    radSize = 2

    goal = [goalX,dataSize]
    goal2 = [int(dataSize * 0.75),dataSize]
    goal3 = [int(dataSize * 0.25),dataSize]

    setPoin = 200


    addFactor = 0.5
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
    print("X: ")
    print(" ")
    xVector0 = Value("f", 0)
    xVector1 = Value("f", 0)
    process1 = Process(target=getLeanX, args=((newImageX, radSize, addFactor, seek_points, originPoint, xVector0, xVector1)))
    #xVector, totalXInt = getLeanX(newImageX, radSize, addFactor, seek_points, originPoint)
    print("Y: ")
    print(" ")
    yVector0 = Value("f", 0)
    yVector1 = Value("f", 0)
    #process2 = Process(target=getLeanY, args=((newImageY, radSize, addFactor, seek_points, originPoint, yVector0, yVector1)))
    #yVector, totalYInt = getLeanY(newImageY, radSize, addFactor, seek_points, originPoint)


    process1.start()
    #process2.start()
    process1.join()
    #process2.join()

    xVector = [float(xVector0.value), float(xVector1.value)]
    yVector = [float(yVector0.value), float(yVector1.value)]
    #xVector = [0, 0]
    #yVector = [0, 0]
    #xVector[0] = xVector[0] / len(newImageX)
    #totalXInt = totalXInt / len(newImageX)
    print(xVector, yVector)

    print("   ")
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
xD, yD, zD = getDirection(dispser)
print(f"Total vector:  {xD}  {yD}  {zD}")
cv2.imshow('gray', dispser)
#plt.show()
cv2.waitKey()


#print(disps.shape)
#cv2.imshow("J", disps)
#cv2.waitKey()
