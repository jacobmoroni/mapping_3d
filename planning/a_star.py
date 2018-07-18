"""

A* grid based planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
See also code of Christian Careaga (http://code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/)

"""


# #this removes python2.7 paths so it wont screw everything up
# import sys
# dir_remove = []
# for p in sys.path:
    # if p.find('python2') !=-1:
        # dir_remove.append(p)
# for p in dir_remove:
    # sys.path.remove(p)

import matplotlib.pyplot as plt
import math
import cv2
import numpy as np
from ipdb import set_trace

show_animation = True


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
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]
    
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
    # if node.y<len(obmap[0]):
        # if node.x<len(obmap):
        # print node.x 
        # print node.y
        
    if obmap[int(node.y)][int(node.x)]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):
    
    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    print("minx:", minx)
    print("miny:", miny)
    print("maxx:", maxx)
    print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    print("xwidth:", xwidth)
    print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(int(xwidth))] for i in range(int(ywidth))]
    obmap = np.array(obmap)
    print np.array(obmap).shape    # set_trace()
    for ix in range(int(xwidth)-1):
        x = ix + minx
        for iy in range(int(ywidth)-1):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    # set_trace()
                    # if iy < len(obmap[0]): 
                    print ix,iy
                    obmap[iy][ix] = True   
                    break
    # set_trace()
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


def main():
    print(__file__ + " start!!")

    # start and goal position
    # sx = 5.0  # [m]
    # sy = 25.0  # [m]
    # gx = 10.0  # [m]
    # gy = 5.0  # [m]
    sx = 25.0  # [m]
    sy = 5.0  # [m]
    gx = 5.0  # [m]
    gy = 10.0  # [m]
    # sx = 2.0  # [m]
    # sy = 5.0  # [m]
    # gx = 5.3  # [m]
    # gy = 1.3  # [m]
    grid_size = 0.1  # [m]
    # grid_size = 0.05
    robot_size = 0.2  # [m]

    ox, oy = generate_obstacles('lab_map.png')
    # ox, oy = generate_obstacles('maze1.jpg')
    # set_trace()
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
    # set_trace()
    px_conv=0.03103

    
    print ("xmin: ") 
    print (min(obs[:,0])*px_conv)
    print ("xmax: ")
    print (max(obs[:,0])*px_conv)
    print ("ymin: ")
    print (min(obs[:,1])*px_conv)
    print ("ymax: ")
    print (max(obs[:,1])*px_conv)

    

    #convert pixels to meters
    # px_conv=0.03103
    xmin = min(obs[1,:])*px_conv
    xmax = max(obs[1,:])*px_conv
    ymin = min(obs[0,:])*px_conv
    ymax = max(obs[0,:])*px_conv

    ob_list = []
    ox = []
    oy = [] 
    
    # ox = obs[1]
    # oy = obs[0]

    # prox_thresh = 0.2*2 
    prox_thresh = 0.1*2
    # prox_thresh = 0.1
    for i,x in enumerate(obs.T):
        x = x*px_conv
        if i == 0:
            ob_list.append((x[1],x[0],0.25))
            ox.append(x[0])
            oy.append(x[1])
            print ("Generating Map:")
           # ob_list.append((x[1],x[0],0.15))
        else:
            if i%1000 == 0:
                print (np.float(i)/len(obs.T)*100),'%'
            min_dist = 1
            for i in range(0,len(ob_list)):
                min_dist =min(np.sqrt((x[1]-ob_list[i][0])**2+(x[0]-ob_list[i][1])**2),min_dist)
            if min_dist > prox_thresh:
                ob_list.append((x[1],x[0],0.25))
                ox.append(x[0])
                oy.append(x[1])
    return ox, oy
    # return ob_list,xmin,xmax,ymin,ymax

if __name__ == '__main__':
    main()
