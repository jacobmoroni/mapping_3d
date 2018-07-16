"""
Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai(@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import random
import math
import copy

# from ipdb import set_trace
import numpy as np
import cv2

show_animation = True


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea, expandDis=0.1, goalSampleRate=20, maxIter=500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minxrand = randArea[0]
        self.maxxrand = randArea[1]
        self.minyrand = randArea[2]
        self.maxyrand = randArea[3]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minxrand, self.maxxrand), random.uniform(
                    self.minyrand, self.maxyrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacleList):
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")
        for (x, y, size) in self.obstacleList:
            self.PlotCircle(x, y, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        # plt.axis([0, 15, 0, 30])
        # plt.axis([1, 6, 1, 5])
        plt.axis([self.minxrand,self.maxxrand,self.minyrand,self.maxyrand])
        plt.grid(True)
        plt.pause(0.01)

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-k")

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def GetPathLength(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

    return le


def GetTargetPoint(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen
    #  print(partRatio)
    #  print((ti,len(path),path[ti],path[ti+1]))

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio
    #  print((x,y))

    return [x, y, ti]


def LineCollisionCheck(first, second, obstacleList):
    # Line Equation

    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
        if d <= (size):
            return False

    #  print("OK")

    return True  # OK


def PathSmoothing(path, maxIter, obstacleList):
    #  print("PathSmoothing")

    le = GetPathLength(path)

    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        #  print(pickPoints)
        first = GetTargetPoint(path, pickPoints[0])
        #  print(first)
        second = GetTargetPoint(path, pickPoints[1])
        #  print(second)

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not LineCollisionCheck(first, second, obstacleList):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = GetPathLength(path)

    return path


def main():
    # ====Search Path with RRT====
    # Parameter
    obstacleList,xmin,xmax,ymin,ymax = generate_obstacles('lab_map.png')  # [x,y,size]
    # obstacleList,xmin,xmax,ymin,ymax = generate_obstacles('maze1.jpg')  # [x,y,size]
    rrt = RRT(start=[5, 25], goal=[10, 5],
    # rrt = RRT(start=[2, 5], goal=[5.3, 1.3],
              randArea=[xmin, xmax, ymin, ymax], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

    # Path smoothing
    maxIter = 1000
    smoothedPath = PathSmoothing(path, maxIter, obstacleList)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.plot([x for (x, y) in smoothedPath], [
            y for (x, y) in smoothedPath], '-b')

        plt.grid(True)
        plt.pause(0.001)  # Need for Mac
        plt.show()

def generate_obstacles(file):
    map_raw = cv2.imread(file,0)
    # cv2.imshow('raw',map_raw)
    
    thresh = 10
    # thresh = 50  #10
    map_bw = cv2.threshold(map_raw, thresh, 255, cv2.THRESH_BINARY)[1]
    map_bw = cv2.bitwise_not(map_bw)
    # cv2.imshow ('bw',map_bw)

    #try to clean up noise in the map
    kernel = np.ones((5,5),np.uint8)
    # map_bw = cv2.dilate(map_bw,kernel,iterations = 3)
    # map_bw = cv2.erode(map_bw,kernel,iterations = 3)
    map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_CLOSE,kernel)
    map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_CLOSE,kernel)
    # map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_OPEN,kernel)
    # cv2.imshow('bw_d',map_bw)


    #convert to array to begin generating obstacles
    map_mat = np.array(map_bw)
    obs = np.nonzero(map_mat)
    obs = np.array(obs)
    px_conv=0.03103
    print "xmin: " 
    print min(obs[:,0])*px_conv
    print "xmax: "
    print max(obs[:,0])*px_conv
    print "ymin: "
    print min(obs[:,1])*px_conv
    print "ymax: "
    print max(obs[:,1])*px_conv

    

    #convert pixels to meters
    # px_conv=0.03103
    xmin = min(obs[1,:])*px_conv
    xmax = max(obs[1,:])*px_conv
    ymin = min(obs[0,:])*px_conv
    ymax = max(obs[0,:])*px_conv

    ob_list = []
    prox_thresh = 0.2*2
    # prox_thresh = 0.09*2
    for i,x in enumerate(obs.T):
        x = x*px_conv
        if i == 0:
            ob_list.append((x[1],x[0],0.25))
            # ob_list.append((x[1],x[0],0.15))
        else:
            print (np.float(i)/len(obs.T))
            min_dist = 1
            for i in range(0,len(ob_list)):
                min_dist =min(np.sqrt((x[1]-ob_list[i][0])**2+(x[0]-ob_list[i][1])**2),min_dist)
            if min_dist > prox_thresh:
                ob_list.append((x[1],x[0],0.25))
    
    # obs_dict = dixt()
    # obs_dict['obstacles'] = ob_list 
    # sio.savemat(ob_list)
    # xmin = 0
    # xmax = 15
    # ymin = 0
    # ymax = 30
    # xmin = 1
    # xmax = 6
    # ymin = 1
    # ymax = 5
    return ob_list,xmin,xmax,ymin,ymax
if __name__ == '__main__':
    main()
