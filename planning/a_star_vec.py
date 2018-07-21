"""

A* grid based planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
See also code of Christian Careaga (http://code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/)

"""

import matplotlib.pyplot as plt
import math
import cv2

from ipdb import set_trace
import numpy as np
import cv2

show_animation = True
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


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_fianl_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)

    ox = ox / reso
    oy = oy / reso

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        # show graph
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id in closedset:
                continue

            if n_id not in openset:
                openset[n_id] = node

            # The distance from start to a neighbor.
            # The "dist_between" function may vary as per the solution requirements.
            if node.cost >= openset[n_id].cost:
                continue  # This is not a better path.

            # This path is the best until now. Record it!
            openset[n_id].cost = node.cost
            openset[n_id].pind = c_id

    rx, ry = calc_fianl_path(ngoal, closedset, reso)

    return rx, ry


def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[int(node.x)][int(node.y)]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx+1)
    ywidth = round(maxy - miny+1)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)
    # set_trace()
    # obstacle map generation
    obmap = np.zeros([int(xwidth),int(ywidth)])
    #TODO vectorize this it takes way too long
    # obmap = [[False for i in range(int(xwidth))] for i in range(int(ywidth))]

    # for ix in range(int(xwidth)):
    #     x = ix + minx
    #     for iy in range(int(ywidth)):
    #         y = iy + miny
    #         #  print(x, y)
    #         for iox, ioy in zip(ox, oy):
    #             d = math.sqrt((iox - x)**2 + (ioy - y)**2)
    #             if d <= vr / reso:
    #                 obmap[ix][iy] = True
    #                 break
    obmap[ox.astype(int)-int(minx),oy.astype(int)-int(miny)]=1

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion

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
    grid_size = .1
    # start and goal position
    # sx = 10.0  # [m]
    # sy = 10.0  # [m]
    # gx = 50.0  # [m]
    # gy = 50.0  # [m]
    # grid_size = 1.0  # [m]
    # robot_size = 1.0  # [m]

    # ox, oy = [], []

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
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()



if __name__ == '__main__':
    main()
