"""
Path Planning Classes with Randomized Rapidly-Exploring Random Trees (RRT)

@author: Jacob Olson
based off of code written by AtsushiSakai(@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import random
import math
import copy

from ipdb import set_trace
import numpy as np
import cv2

'''
Tunable parameters that get passed in to the functions

show_animation ---- show the rrt growing
show_visualization- show the final path and select with mouse
file -------------- file containing the map to be loaded in
bw_thresh --------- black/white threshold used for extracting obstacles
obstacleSize ------ radius of obstacles in map
px_conv ----------- conversion of meters/pixel
thetadeg ---------- how many degrees to rotate the obstacle map (for adjusting to global map)
x_shift ----------- how much to shift the map in the x direction
y_shift ----------- how much to shift the map in the y direction
z ----------------- Z value for waypoints generated
unknown_as_obs ---- Sets all unknown grey area in RTAB-Map occupancy grid to obstacles
show_obs_size ----- Show Obstacles with radius as circles or without as constant size dots
waypoint_thresh --- Threshold Distance to prune waypoints at the expandDisi

expandDis --------- Distance to expand the RRT tree each iteration
goalSampleRate ---- Percent chance of "randomly" sampling goal for RRT
maxIter ----------- Number of times the path smoothing iterates

'''

class RRTSearch:
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, searchArea, obstacleList, show_obs_size,
            expandDis, goalSampleRate, maxIter):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        searchArea:Ramdom Samping Area [min,max]

        """
        self.start = [start[0],start[1],None]
        self.goal = [goal[0],goal[1],None]
        self.minxrand = searchArea[0]
        self.maxxrand = searchArea[1]
        self.minyrand = searchArea[2]
        self.maxyrand = searchArea[3]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.show_obs_size = show_obs_size

    def Planning(self, show_visualization, animation=False):
        """
        Pathplanning

        vizualization: flag for visualization on or off
        animation: flag for animation on or off
        """
        print "Finding Path"
        first_time = True
        self.nodeList = [self.start]
        i = 0
        while True:
            if first_time:
                if show_visualization:
                    #Replot before starting
                    self.DrawGraph()
                first_time = False

            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minxrand, self.maxxrand), random.uniform(
                    self.minyrand, self.maxyrand)]
            else:
                rnd = [self.goal[0], self.goal[1]]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode[1], rnd[0] - nearestNode[0])

            newNode = copy.deepcopy(nearestNode)
            newNode[0] += self.expandDis * math.cos(theta)
            newNode[1] += self.expandDis * math.sin(theta)
            newNode[2] = nind

            if not self.__CollisionCheck(self.goal,self.obstacleList):
                print "bad goal point"

            if not self.__CollisionCheck(newNode, self.obstacleList):
                continue

            self.nodeList.append(newNode)

            if self.LineCollisionCheck(newNode,self.goal,self.obstacleList):
                break
            # check goal
            dx = newNode[0] - self.goal[0]
            dy = newNode[1] - self.goal[1]
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                break
            # print i
            if animation:
                self.DrawGraph(rnd)
            i += 1
        #add nodes in final path to path variable
        path = [[self.goal[0], self.goal[1]]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex][2] is not None:
            node = self.nodeList[lastIndex]
            path.append([node[0], node[1]])
            lastIndex = node[2]
        path.append([self.start[0], self.start[1]])

        return path

    def DrawGraph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")

        for node in self.nodeList:
            if node[2] is not None:
                plt.plot([node[0], self.nodeList[node[2]][0]], [
                         node[1], self.nodeList[node[2]][1]], "-g")
        if self.show_obs_size:
            self.PlotCircle(self.obstacleList[:,0],self.obstacleList[:,1],self.obstacleList[:,2])

        else:
            plt.plot(self.obstacleList[:,0],self.obstacleList[:,1],".k")

        plt.plot(self.start[0], self.start[1], "xr")
        plt.plot(self.goal[0], self.goal[1], "xr")
        plt.xlim(self.minxrand,self.maxxrand)
        plt.ylim(self.minyrand,self.maxyrand)
        plt.axis('scaled')
        plt.grid(True)
        # plt.pause(0.01)

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 30))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-k")

    def GetNearestListIndex(self, nodeList, rnd):
        #find nearest node to randomly sampled number and return index of node
        nodeList = np.array(nodeList)
        dlist = (nodeList[:,0]-rnd[0]) **2 + (nodeList[:,1]-rnd[1])**2
        minind = np.argmin(dlist)
        return minind

    def __CollisionCheck(self, node, obstacleList):
        #Check for endpoint collisions with new node
        dobs = obstacleList - [node[0],node[1],0]
        dist = np.sqrt(dobs[:,0]**2+dobs[:,1]**2)
        size = obstacleList[:,2]
        if min(dist-size)<0:
            return False #collision
        else:
            return True #safe

    def LineCollisionCheck(self,first, second, obstacleList):
        # Uses Line Equation to check for collisions along new line made by connecting nodes

        x1 = first[0]
        y1 = first[1]
        x2 = second[0]
        y2 = second[1]

        try:
            a = y2 - y1
            b = x2 - x1
            c = x2*y1 - y2*x1
        except ZeroDivisionError:
            return False
        dist = abs(a*obstacleList[:,0]-b*obstacleList[:,1]+c)/np.sqrt(a*a+b*b)-obstacleList[:,2]

        #filter to only look at obstacles within range of endpoints of lines
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

class RRTSmooth:
    """
    Class for RRT path smoothing
    """
    def __init__(self):
        self.this = 0

    def GetPathLength(self, path):
        # return current length of path
        path = np.array(path)
        dpath = path - np.roll(path,1,axis=0)
        dlen = np.sqrt(dpath[:,0]**2+dpath[:,1]**2)
        le = sum(dlen[1:len(dlen)])

        return le


    def GetTargetPoint(self, path, targetL):
        #return location and position of nearest node to randomly sampled length
        path = np.array(path)
        dpath = path - np.roll(path,1,axis=0)
        dlen = np.sqrt(dpath[:,0]**2+dpath[:,1]**2)
        dlen = dlen[1:len(dlen)]
        dsum = np.cumsum(dlen)
        ti = np.argmax(dsum>=targetL)-1
        lastPairLen = dlen[ti+1]
        le = dsum[ti+1]

        partRatio = (le - targetL) / lastPairLen

        x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
        y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

        return [x, y, ti]


    def LineCollisionCheck(self,first, second, obstacleList):
        # Uses Line Equation to check for collisions along new line made by connecting nodes

        x1 = first[0]
        y1 = first[1]
        x2 = second[0]
        y2 = second[1]

        try:
            a = y2 - y1
            b = x2 - x1
            c = x2*y1 - y2*x1
        except ZeroDivisionError:
            return False
        dist = abs(a*obstacleList[:,0]-b*obstacleList[:,1]+c)/np.sqrt(a*a+b*b)-obstacleList[:,2]

        #filter to only look at obstacles within range of endpoints of lines
        prox = np.bitwise_not(np.bitwise_and(
                np.bitwise_or(
                    np.bitwise_and(obstacleList[:,0]<=x2 ,obstacleList[:,0]<=x1),
                    np.bitwise_and(obstacleList[:,0]>=x2,obstacleList[:,0]>=x1)),
                np.bitwise_or(
                    np.bitwise_and(obstacleList[:,1]<=y2,obstacleList[:,1]<=y1),
                    np.bitwise_and(obstacleList[:,1]>=y2,obstacleList[:,1]>=y1))))
        
        # plt.plot([x1,x2],[y1,y2],'g')
        # plt.plot(obstacleList[prox,0],obstacleList[prox,1],'.k')
        # plt.show()
        if dist[prox].size > 0:
            if min(dist[prox])<=0:
                return False
            else:
                return True

    def PathSmoothing(self, path, maxIter, obstacleList):
        print "Smoothing Path"
        le = self.GetPathLength(path)

        
        pidx = 0
        while pidx < len(path)-1:
            # set_trace()
            pidx +=1
            for i in range(1,len(path)-pidx):
                first = [path[pidx][0],path[pidx][1],pidx]
                second = [path[len(path)-i][0],path[len(path)-i][1],len(path)-i]
                if not self.LineCollisionCheck(first,second,obstacleList):
                    continue
                    if first[2]+1 == second[2]:
                        # pidx+=1
                        set_trace()
                else:
                    # Create New path
                    newPath = []
                    newPath.extend(path[:first[2] + 1])
                    # newPath.append([first[0], first[1]])
                    newPath.append([second[0], second[1]])
                    newPath.extend(path[second[2] + 1:])
                    path = newPath
                    # pidx +=1
                    break
        
        #randomly sample maxIter times to smooth path
        # for i in range(maxIter):

            # # Sample two points
            # pickPoints = [random.randint(0, len(path)-1), random.randint(0, len(path)-1)]
            # pickPoints.sort()
            # first = [path[pickPoints[0]][0],path[pickPoints[0]][1],pickPoints[0]]
            # second = [path[pickPoints[1]][0],path[pickPoints[1]][1],pickPoints[1]] 
            
            # # # Sample two points
            # # pickPoints = [random.uniform(0, le), random.uniform(0, le)]
            # # pickPoints.sort()
            # # first = self.GetTargetPoint(path, pickPoints[0])
            # # second = self.GetTargetPoint(path, pickPoints[1])

            # # if first[2] <= 0 or second[2] <= 0:
                # # continue

            # if (second[2] + 1) > len(path):
                # continue

            # if second[2] == first[2]:
                # continue

            # # collision check
            # if not self.LineCollisionCheck(first, second, obstacleList):
                # continue
            # # Create New path
            # newPath = []
            # newPath.extend(path[:first[2] + 1])
            # # newPath.append([first[0], first[1]])
            # newPath.append([second[0], second[1]])
            # newPath.extend(path[second[2] + 1:])
            # path = newPath
            # le = self.GetPathLength(path)

        return path

    def GenerateWaypoints(self, smoothedPath,z,waypoint_thresh):
        #generates waypoints to fly from smoothed path

        waypoints = []
        smoothedPath = np.array(smoothedPath)
        smoothedPath = np.flip(smoothedPath,0)

        #start by filtering out path points within threshold of previous point
        dspath = smoothedPath - np.roll(smoothedPath,1,axis=0)
        dlen = np.sqrt(dspath[:,0]**2+dspath[:,1]**2)
        dlen = dlen[1:len(dlen)]
        smoothedPath = np.delete(smoothedPath,np.where(dlen<waypoint_thresh),0)

        #compute heading angle to fly waypoint (currently assuming flight pointing at next waypoint)
        dsmoothedPath = smoothedPath -np.roll(smoothedPath,1,0)
        smoothedPath = smoothedPath[1:len(smoothedPath)]
        dsmoothedPath = dsmoothedPath[1:len(dsmoothedPath)]
        angle_path = np.arctan2(dsmoothedPath[:,0],dsmoothedPath[:,1])

        #current waypoint follower goes to psi delayed be 1 waypoint so shift psi back by 1
        angle_path = np.roll(angle_path,-1,0)
        if len(angle_path)>1:
            angle_path[-1] = angle_path[-2]


        #waypoints are [N,E,D,psi]
        waypoints = np.array([smoothedPath[:,1],smoothedPath[:,0],z*np.ones(len(dsmoothedPath)),angle_path]).T

        return waypoints

class GenerateMap:
    def __init__(self, points, file, px_conv, bw_thresh, obstacleSize, unknown_as_obs, show_obs_size,
                show_visualization, theta, x_shift, y_shift, pre_gen_obs, robotx_g, roboty_g):
        # Pass in None as file to use obstacles generated from ROS
        self.points = points
        self.show_visualization = show_visualization
        if self.show_visualization:
            self.cid = points.figure.canvas.mpl_connect('button_press_event', self)
        self.node = 0
        self.theta = theta
        self.x_trans = x_shift
        self.y_trans = y_shift
        self.px_conv = px_conv
        self.bw_thresh = bw_thresh
        self.file = file
        self.ob_size = obstacleSize
        self.show_obs_size = show_obs_size


        if self.file == 0:
            self.ROSObstacles = True
            self.obs = pre_gen_obs
            self.robotx = robotx_g
            self.roboty = roboty_g
            self.GenerateObstaclesFromROS()
        else:
            self.ROSObstacles = False
            self.GenerateObstaclesFromImg()

        self.RotObs()
        self.TransObs()
        #find min and max on map
        self.xmin = min(self.obs[:,0])
        self.xmax = max(self.obs[:,0])
        self.ymin = min(self.obs[:,1])
        self.ymax = max(self.obs[:,1])

        if self.show_visualization:
            self.xs = [self.xmin,self.ymin]
            self.ys = [self.xmin,self.ymin]
            self.start = [self.xmin,self.ymin]
            self.goal = [self.xmin,self.ymin]
            self.PlotObs()

    def __call__(self, event):
        #Records x and y locations of mouse clicks and sends them to start and goal positions
        if event.inaxes!=self.points.axes: return

        if self.ROSObstacles:
            self.xs[0] = self.robotx
            self.ys[0] = self.roboty
            self.xs[1] = event.xdata
            self.ys[1] = event.ydata
            self.start = [self.robotx,self.roboty]
            self.goal = [event.xdata,event.ydata]
            print "goal selected: x= ",event.xdata," y= ", event.ydata

        else:
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

        self.points.set_data(self.xs, self.ys)
        self.points.figure.canvas.draw()
        return self.start,self.goal

    def GenerateObstaclesFromROS(self):
        sizes = np.ones([1,len(self.obs)])*self.ob_size
        self.obs = np.append(self.obs.T,sizes,axis=0).T

    def GenerateObstaclesFromImg(self):
        if unknown_as_obs:
            map_raw = cv2.imread(self.file,1)
        else:
            map_raw = cv2.imread(self.file,0)

        map_bw = cv2.threshold(map_raw, self.bw_thresh, 255, cv2.THRESH_BINARY)[1]
        map_bw = cv2.bitwise_not(map_bw)

        #try to clean up noise in the map
        # map_bw = cv2.dilate(map_bw,kernel,iterations = 3)
        # map_bw = cv2.erode(map_bw,kernel,iterations = 3)
        # map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_CLOSE,kernel)
        # map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_OPEN,kernel)
        # map_bw = cv2.morphologyEx(map_bw, cv2.MORPH_CLOSE,kernel)

        if unknown_as_obs:
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

    def RotObs(self):
        #rotate obstacles by theta
        rot_theta = np.array([[np.cos(self.theta),-np.sin(self.theta)],
                              [np.sin(self.theta), np.cos(self.theta)]])
        self.obs[:,0:2] = np.matmul(self.obs[:,0:2],rot_theta)

    def TransObs(self):
        #translate obstacles by x_shift and y_shift
        self.obs[:,0]+=self.x_trans
        self.obs[:,1]+=self.y_trans

    def PlotObs(self):
        #plot obstacles for visualization
        if self.show_obs_size:
            self.PlotCircle(self.obs[:,0],self.obs[:,1],self.obs[:,2])
        else:
            plt.plot(self.obs[:,0],self.obs[:,1],".k")
        plt.xlim(self.xmin,self.xmax)
        plt.ylim(self.ymin,self.ymax)
        plt.axis('scaled')
        plt.grid(True)
        # plt.pause(0.01)

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
    generator = GenerateMap(line, file, px_conv, bw_thresh, obstacleSize,theta,x_shift,y_shift)
    plt.show()

    start = generator.start
    goal =  generator.goal
    obstacleList = generator.obs
    xmin = generator.xmin
    ymin = generator.ymin
    xmax = generator.xmax
    ymax = generator.ymax

    rrt = RRT(start, goal, [xmin, xmax, ymin, ymax],
              obstacleList, expandDis, goalSampleRate, maxIter)
    path = rrt.Planning(animation=show_animation)

    # Path smoothing
    smoothedPath = PathSmoothing(path, maxIter, obstacleList)
    waypoints = GenerateWaypoints(smoothedPath,z,waypoint_thresh)
    # Draw final path
    if plot_final:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.plot([x for (x, y) in smoothedPath], [
            y for (x, y) in smoothedPath], '-b')
        plt.plot(waypoints[:,1],waypoints[:,0],'oc')
        plt.plot([waypoints[:,1],waypoints[:,1]+.5*np.cos(-waypoints[:,3]+np.pi/2)],
                 [waypoints[:,0],waypoints[:,0]+.5*np.sin(-waypoints[:,3]+np.pi/2)],'-c')
        plt.grid(True)
        plt.pause(0.001)
        plt.show()
        set_trace()
if __name__ == '__main__':
    main()
