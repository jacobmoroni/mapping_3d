"""
Path Planning Classes with Randomized Rapidly-Exploring Random Trees (RRT)

@author: Jacob Olson
based off of code written by AtsushiSakai(@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import pygame
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
        Setting Parameters

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

        plt.plot(self.obstacleList[:,0],self.obstacleList[:,1],".k")

        plt.plot(self.start[0], self.start[1], "xr")
        plt.plot(self.goal[0], self.goal[1], "xr")
        plt.xlim(self.minxrand,self.maxxrand)
        plt.ylim(self.minyrand,self.maxyrand)
        plt.axis('scaled')
        plt.grid(True)
        # plt.pause(0.01)


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
                        set_trace()
                else:
                    # Create New path
                    newPath = []
                    newPath.extend(path[:first[2] + 1])
                    newPath.append([second[0], second[1]])
                    newPath.extend(path[second[2] + 1:])
                    path = newPath
                    break
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

def main():
    print "loaded"
if __name__ == '__main__':
    main()
