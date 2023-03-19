from Original_apf import APF, Vector2d
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Circle
import random
import time



def check_vec_angle(v1: Vector2d, v2: Vector2d):
    v1_v2 = v1.deltaX * v2.deltaX + v1.deltaY * v2.deltaY
    angle = math.acos(v1_v2 / (v1.length * v2.length)) * 180 / math.pi
    return angle


class APF_Improved(APF):
    def __init__(self, rList, rad, rS, start: (), goal: (), obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = is_plot
        self.delta_t = 0.001
        self.rList = rList
        self.rS = rS
        self.rad = rad
        radUse = []
        for jn in range(0, len(self.rList)):
            #print(len(self.rList))
            #print(self.rList[jn] - (self.rad * 0.2))
            rM = abs(self.rList[jn] - (self.rad * 0.1)) ** 2.5
            radUse.append(rM)
        self.radUse = radUse
        #print('Rads {0}'.format(self.rList))
        #print('RadsUse {0}'.format(radUse))
        #print("New: ")
        #print(self.obstacles)
    def repulsion(self):
        """
        斥力计算, 改进斥力函数, 解决不可达问题
        :return: 斥力大小
        """
        rep = Vector2d(0, 0)
        for obst in range(len(self.obstacles)):
            obstacle = self.obstacles[obst]
            #hv = self.rr
            hv = self.rList[obst]
            rT = self.radUse[obst]
            repConst = hv
            #repConst = self.k_rep
            #print(hv)
            # obstacle = Vector2d(0, 0)
            obs_to_rob = self.current_pos - obstacle
            rob_to_goal = self.goal - self.current_pos
            if (obs_to_rob.length > rT):
                pass
            else:
                rep_1 = Vector2d(obs_to_rob.direction[0], obs_to_rob.direction[1]) * rT * (
                        1.0 / obs_to_rob.length - 1.0 / rT) / (obs_to_rob.length ** 2) * (rob_to_goal.length ** 2)
                rep_2 = Vector2d(rob_to_goal.direction[0], rob_to_goal.direction[1]) * rT * ((1.0 / obs_to_rob.length - 1.0 / rT) ** 2) * rob_to_goal.length
                rep +=(rep_1+rep_2)

        return rep


def mainAvoid(obs, xBias, dataSize, rList, rad, rS):
    originX = (dataSize / 2)
    hg = 0
    xCT = 0
    yCT = 0
    is_plot = False
    #print(xBias)
    if len(obs) >= 1:

        #while True:
        k_att, k_rep = 4.0, 3.0
        rr = 4
        step_size, max_iters, goal_threashold = 1.2, 30, 2.0
        #step_size_ = 100
        start, goal = (originX, 0), ((originX + xBias), dataSize)
        is_plot = False
        if is_plot:
            fig = plt.figure(figsize=(7, 7))
            subplot = fig.add_subplot(111)
            subplot.set_xlabel('X-distance: m')
            subplot.set_ylabel('Y-distance: m')
            subplot.plot(start[0], start[1], '*r')
            subplot.plot(goal[0], goal[1], '*r')
        #vb = rr / 5

        #obs = [[3, 4], [2, 4], [3, 3], [6, 1], [6, 7], [10, 6], [11, 12], [14, 14]]
        #print('obstacles: {0}'.format(obs))
        for i in range(0):
            obs.append([random.uniform(2, goal[1] - 1), random.uniform(2, goal[1] - 1)])

        if is_plot:
            for OBg in range(0, len(obs)):
                OB = obs[OBg]
                newRad = (rList[OBg] - (rad * rS)) ** 2.6
                newR2 = rList[OBg]
                circle = Circle(xy=(OB[0], OB[1]), radius=newR2, alpha=0.3)
                subplot.add_patch(circle)
                subplot.plot(OB[0], OB[1], 'xk')
        # t1 = time.time()
        # for i in range(1000):

        # path plan
        if is_plot:
            apf = APF_Improved(rList, rad, rS, start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
        else:
            apf = APF_Improved(rList, rad, rS, start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)

        apf.path_plan()
        if apf.is_path_plan_success:
            path = apf.path
            #print(path)
            prevY = 0
            prevX = 0

            for hb in path:
                xV = hb[0] - originX
                yV = hb[1]
                if yV < (dataSize * 0.8):
                    xCT += (xV * 1.5)
                    yCT += (yV - prevY)
                    prevY = yV
                    prevX = xV

            """path_ = []
            i = int(step_size_ / step_size)
            while (i < len(path)):
                path_.append(path[i])
                i += int(step_size_ / step_size)
            if path_[-1] != path[-1]:  # 添加最后一个点
                path_.append(path[-1])
            print('planed path points:{}'.format(path_))
            print('path plan success')
            if is_plot:
                px, py = [K[0] for K in path_], [K[1] for K in path_]  # 路径点x坐标列表, y坐标列表
                subplot.plot(px, py, '^k')
                plt.show()"""





        else:
            #print('path plan failed')
            xCT = 0
            yCT = 0

    hg += 1
    #print(" ")
    #print("Original x and z: " + str(xCT / 1.8) + ", " + str(yCT))
    #print("Current x and z: " + str(xCT) + ", " + str(yCT))
    #print(" ")

    return xCT, yCT


def testMethod():
    dataSize = 17
    originX = dataSize / 2
    obses = []
    for h in range(0, 10):
        xCord = random.randint(5, 15)
        yCord = random.randint(5, 15)
        if xCord == originX:
            xCord = random.randint(int(dataSize / 3), int(dataSize / 2))
        currObs = [xCord, yCord]
        obses.append(currObs)
    rList = []
    rr = 4
    rDist = rr / (dataSize - (dataSize * 0.4))
    for hk in range(0, len(obses)):
        jb = rDist * ((dataSize + (dataSize * 0.4)) - obses[hk][1])
        #print(obses[hk], jb)
        rList.append(5)
    #obses = []
    #print(obses)
    #mainAvoid(obses, 0, dataSize, rList)
#testMethod()
