"""
Path Planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai(@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import random
import math
import copy

from ipdb import set_trace
import numpy as np
import cv2

show_animation = False
plot_final = True
file = "lab_map.png"
# file = "maze2.png"
bw_thresh = 10
obstacleSize = .15
px_conv = 0.03103
color_image = False #This sets all unknown grey area to obstacles but slows down plotting
show_obs_size = False #Use this to speed up plotting, by plotting dots rather than circles

if color_image:
    bw_thresh = 90


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea, expandDis=0.15, goalSampleRate=20, maxIter=500):
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

    def Planning(self, animation=False):
        """
        Pathplanning

        animation: flag for animation on or off
        """
        print "Finding Path"
        first_time = True
        self.nodeList = [self.start]
        while True:
            if first_time == True:
                self.DrawGraph()
                first_time = False
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
        if show_obs_size:
            self.PlotCircle(self.obstacleList[:,0],self.obstacleList[:,1],self.obstacleList[:,2])
            # for (x, y, size) in self.obstacleList:
                # self.PlotCircle(x, y, size)
        else:
            plt.plot(self.obstacleList[:,0],self.obstacleList[:,1],".k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([self.minxrand,self.maxxrand,self.minyrand,self.maxyrand])
        plt.grid(True)
        plt.pause(0.01)

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 30))
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
        dobs = obstacleList - [node.x,node.y,0]
        dist = np.sqrt(dobs[:,0]**2+dobs[:,1]**2)
        size = obstacleList[:,2]
        if min(dist-size)<0:
            return False #collision
        else:
            return True #safe

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
        b = x2 - x1
        # c = y2 * (x2 - x1) - x2 * (y2 - y1)
        c = x2*y1 - y2*x1
    except ZeroDivisionError:
        return False
    dist = abs(a*obstacleList[:,0]-b*obstacleList[:,1]+c)/np.sqrt(a*a+b*b)-obstacleList[:,2]
    prox = np.bitwise_not(np.bitwise_and(
            np.bitwise_or(
                np.bitwise_and(obstacleList[:,0]<=x2 ,obstacleList[:,0]<=x1),
                np.bitwise_and(obstacleList[:,0]>=x2,obstacleList[:,0]>=x1)),
            np.bitwise_or(
                np.bitwise_and(obstacleList[:,1]<=y2,obstacleList[:,1]<=y1),
                np.bitwise_and(obstacleList[:,1]>=y2,obstacleList[:,1]>=y1))))

    if dist[prox].size > 0:
        if min(dist[prox])<=0:
            return False
        else:
            return True

def PathSmoothing(path, maxIter, obstacleList):
    print "PathSmoothing"
    le = GetPathLength(path)
    # first = [0,0,0]
    # pickPoints = [0,le]
    # while first[2]<len(path):
        # first = GetTargetPoint(path,PickPoints[0])
        # second = GetTargetPoint(path,PickPoints[1])
        # if LineCollisionCheck(first,second,obstacleList)
    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        first = GetTargetPoint(path, pickPoints[0])
        second = GetTargetPoint(path, pickPoints[1])

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


class GenerateMap:
    def __init__(self, line, file, px_conv, bw_thresh, obstacleSize):
        self.line = line
        self.xs = [0,0]
        self.ys = [0,0]
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
        self.node = 0
        self.start = [0,0]
        self.goal = [0,0]
        self.xmin = 0
        self.ymin = 0
        self.xmax = 0
        self.ymax = 0
        self.px_conv = px_conv
        self.bw_thresh = bw_thresh
        self.file = file
        self.ob_size = obstacleSize
        self.generate_obstacles()
        self.PlotObs()

    def __call__(self, event):
        if event.inaxes!=self.line.axes: return

        if self.node == 0:
            self.xs[0] = event.xdata
            self.ys[0] = event.ydata
            self.start=[event.xdata,event.ydata]
            print "starting point selected: x= ",event.xdata," y= ", event.ydata
            self.node = 1
        else:
            self.xs[1] = event.xdata
            self.ys[1] = event.ydata
            self.goal = [event.xdata,event.ydata]
            print "goal selected: x= ",event.xdata," y= ", event.ydata
            self.node = 0
        self.line.set_data(self.xs, self.ys)
        self.line.figure.canvas.draw()
        return self.start,self.goal

    def generate_obstacles(self):
        if color_image:
            map_raw = cv2.imread(self.file,1)
        else:
            map_raw = cv2.imread(self.file,0)

        map_bw = cv2.threshold(map_raw, self.bw_thresh, 255, cv2.THRESH_BINARY)[1]
        map_bw = cv2.bitwise_not(map_bw)

        #try to clean up noise in the map
        kernel = np.ones((5,5),np.uint8)
        # map_bw = cv2.dilate(map_bw,kernel,iterations = 3)
        # map_bw = cv2.erode(map_bw,kernel,iterations = 3)
        map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_CLOSE,kernel)
        # map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_OPEN,kernel)
        map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_CLOSE,kernel)

        if color_image:
            #Clear out color and flatten image
            map_bw[np.where((map_bw == [255,0,255]).all(axis = 2))] = [0,0,0]
            map_bw[np.where((map_bw == [0,255,255]).all(axis = 2))] = [0,0,0]
            map_bw[np.where((map_bw == [255,255,0]).all(axis = 2))] = [0,0,0]
            map_bw[np.where((map_bw == [0,0,255]).all(axis = 2))] = [0,0,0]
            map_bw[np.where((map_bw == [255,0,0]).all(axis = 2))] = [0,0,0]
            map_bw[np.where((map_bw == [0,255,0]).all(axis = 2))] = [0,0,0]

            map_bw = map_bw[:,:,0]

        #convert to array to begin generating obstacles
        map_mat = np.array(map_bw)
        obs = np.nonzero(map_mat)
        obs = np.array(obs)

        #convert pixels to meters
        obs = obs.T*self.px_conv

        #prune obstacles by rounding
        obs = np.round_(obs,1)
        obs = np.unique(obs,axis = 0)

        #add sizes to obstacles
        sizes = np.ones([1,len(obs)])*self.ob_size
        obs = np.append(obs.T,sizes,axis=0).T

        self.obs = obs[:,np.argsort([1,0,2])] #rearange so obstacles are [[x,y,size]]
        self.obs[:,1]=self.obs[:,1]*-1 #invert y to match original image

        #find min and max on map
        self.xmin = min(self.obs[:,0])
        self.xmax = max(self.obs[:,0])
        self.ymin = min(self.obs[:,1])
        self.ymax = max(self.obs[:,1])

        print "xmin: ",self.xmin
        print "xmax: ",self.xmax
        print "ymin: ",self.ymin
        print "ymax: ",self.ymax

        # ob_list = []
        # prox_thresh = 0.2*2
        # # prox_thresh = 0.09*2
        # for i,x in enumerate(obs.T):
            # x = x*px_conv
            # if i == 0:
                # ob_list.append([x[1],x[0],0.25])
                # # ob_list.append((x[1],x[0],0.15))
            # else:
                # print (np.float(i)/len(obs.T))
                # min_dist = 1
                # for i in range(0,len(ob_list)):
                    # min_dist =min(np.sqrt((x[1]-ob_list[i][0])**2+(x[0]-ob_list[i][1])**2),min_dist)
                # if min_dist > prox_thresh:
                    # ob_list.append([x[1],x[0],0.25])
        # select_start(obs,xmin,xmax,ymin,ymax)
         # return obs,xmin,xmax,ymin,ymax

    def PlotObs(self):

        if show_obs_size:
            self.PlotCircle(self.obs[:,0],self.obs[:,1],self.obs[:,2])
        else:
            plt.plot(self.obs[:,0],self.obs[:,1],".k")
        plt.axis([self.xmin,self.xmax,self.ymin,self.ymax])
        plt.grid(True)
        plt.pause(0.01)

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 30))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-k")

def main():

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('Select Start and Goal (Close when finished)')
    line, = ax.plot([0,0], [0,0],"xr")  # empty line
    generator = GenerateMap(line, file, px_conv, bw_thresh, obstacleSize)
    # generator = GenerateMap(line, file = 'lab_map.png',px_conv = 0.03103, bw_thresh = 10)
    plt.show()

    start = generator.start
    goal =  generator.goal
    obstacleList = generator.obs
    xmin = generator.xmin
    ymin = generator.ymin
    xmax = generator.xmax
    ymax = generator.ymax

    rrt = RRT(start, goal,
              randArea=[xmin, xmax, ymin, ymax], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

    # Path smoothing
    maxIter = 1000
    smoothedPath = PathSmoothing(path, maxIter, obstacleList)
    # Draw final path
    if plot_final:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.plot([x for (x, y) in smoothedPath], [
            y for (x, y) in smoothedPath], '-b')

        plt.grid(True)
        plt.pause(0.001)  # Need for Mac
        plt.show()

if __name__ == '__main__':
    main()
