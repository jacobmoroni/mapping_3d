"""

Voronoi Road Map Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt
import cv2
# parameter
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

show_animation = True

plot_final = True
# file = "lab_map.png"
file = "maze2.png"
# file ='maze1.jpg'
bw_thresh = 10
# bw_thresh = 50
obstacleSize = .15
px_conv = 0.03103
color_image = False #This sets all unknown grey area to obstacles but slows down plotting
show_obs_size = False #Use this to speed up plotting, by plotting dots rather than circles

if color_image:
    bw_thresh = 90



class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost 
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        u"""
        Search NN

        inp: input data, single frame or multi frame

        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        u"""
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


def VRM_planning(sx, sy, gx, gy, ox, oy, rr):

    obkdtree = KDTree(np.vstack((ox, oy)).T)

    sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree)
    if show_animation:
        plt.plot(sample_x, sample_y, ".y")

    road_map = generate_roadmap(sample_x, sample_y, rr, obkdtree)

    rx, ry = dijkstra_planning(
        sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx**2 + dy**2)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    nstep = round(d / D)

    for i in range(int(nstep)):
        idxs, dist = okdtree.search(np.matrix([x, y]).T)
        if dist[0] <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    idxs, dist = okdtree.search(np.matrix([gx, gy]).T)
    if dist[0] <= rr:
        return True  # collision

    return False  # OK


def generate_roadmap(sample_x, sample_y, rr, obkdtree):
    """
    Road map generation

    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obkdtree: KDTree object of obstacles
    """

    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(int(nsample)), sample_x, sample_y):

        index, dists = skdtree.search(
            np.matrix([ix, iy]).T, k=nsample)
        inds = index[0][0]
        edge_id = []
        #  print(index)

        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obkdtree):
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[len(road_map) - 2] = nstart

    while True:
        if len(openset) == 0:
            print("Cannot find path")
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]

        # show graph
        if show_animation and len(closedset.keys()) % 2 == 0:
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.sqrt(dx**2 + dy**2)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # generate final course
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):

    for i in range(len(road_map)):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree):
    oxy = np.vstack((ox, oy)).T

    # generate voronoi point
    vor = scipy.spatial.Voronoi(oxy)
    sample_x = [ix for [ix, iy] in vor.vertices]
    sample_y = [iy for [ix, iy] in vor.vertices]

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y



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

    def PlotObs(self):
        # for (x, y, size) in self.obs:
            # plt.plot(x,y,".k")
        plt.plot(self.obs[:,0],self.obs[:,1],".k")
        plt.axis([self.xmin,self.xmax,self.ymin,self.ymax])
        plt.grid(True)
        plt.pause(0.01)



def main():
    print(__file__ + " start!!")


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

    sx = start[0]
    sy = start[1]
    gx = goal[0]
    gy = goal[1]
    robot_size = 0.1

    ox = obstacleList[:,0]
    oy = obstacleList[:,1]

    # start and goal position
    # sx = 10.0  # [m]
    # sy = 10.0  # [m]
    # gx = 50.0  # [m]
    # gy = 50.0  # [m]
    # robot_size = 1.0  # [m]

    # ox = []
    # oy = []

    # for i in range(60):
        # ox.append(i)
        # oy.append(0.0)
    # for i in range(60):
        # ox.append(60.0)
        # oy.append(i)
    # for i in range(61):
        # ox.append(i)
        # oy.append(60.0)
    # for i in range(61):
        # ox.append(0.0)
        # oy.append(i)
    # for i in range(40):
        # ox.append(20.0)
        # oy.append(i)
    # for i in range(40):
        # ox.append(40.0)
        # oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.grid(True)
        # plt.axis("equal")
        plt.axis([xmin,xmax,ymin,ymax])

    rx, ry = VRM_planning(sx, sy, gx, gy, ox, oy, robot_size)

    assert len(rx) != 0, 'Cannot find path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
