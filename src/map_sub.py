import numpy as np
import rospy, tf
import cv2
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from tf.transformations import euler_from_quaternion
import tf
from rrt_planner import GenerateMap, RRTSmooth, RRTSearch
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile

maxIter = 500
obstacleSize = 0.8
expandDis = 0.35
goalSampleRate = 20
z = -1.3
waypoint_thresh = 0.9
color_image = True
show_obs_size = False
waypoint_thresh = 0.09
show_animation = False
plot_final = True

class MapMaker():
    def __init__(self):
        self.reso = 0
        self.mapw = 0
        self.maph = 0
        self.originx = 0
        self.originy = 0
        self.map1d = []
        self.map2d = []
        self.robotx = None
        self.roboty = None
        self.go = True
        self.listener = tf.TransformListener()

        self.gridmap_sub_ = rospy.Subscriber('rtabmap/grid_map', OccupancyGrid, 
                self.gridmapCallback, queue_size=5)
        self.new_waypoint = rospy.ServiceProxy('/slammer/add_waypoint', AddWaypoint)
    def get_transform(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
            # print trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("cannot get tf transform")
            return
        self.robotx = trans[0]
        self.roboty = trans[1]
        print self.robotx , self.roboty

    def gridmapCallback(self,msg):
        self.reso = msg.info.resolution
        self.mapw = msg.info.width
        self.maph = msg.info.height
        self.originx = msg.info.origin.position.x
        self.originy = msg.info.origin.position.y
        self.map1d = np.array(msg.data)
        self.map2d = self.map1d.reshape(self.maph,self.mapw)

        obs = np.where(self.map2d==100)
        # obs = np.where(np.logical_or(self.map2d==100, self.map2d==-1))
        obs = np.array(obs)
        obs  = obs.T*self.reso
        obs[:,0] += self.originy
        obs[:,1] += self.originx
        self.obs = obs
        self.get_transform()

        if self.robotx is not None:
            self.gridmap_sub_.unregister()
            self.go = False

def main():
    rospy.init_node('MapMaker', anonymous=True)
    mapmaker = MapMaker()
    # mapmaker.get_transform()
    while mapmaker.go:
        rospy.rostime.wallsleep(0.1)
    obs = mapmaker.obs

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('Select Goal (Close when finished)')


    file = None
    px_conv = mapmaker.reso
    bw_thresh = 10

    theta = 0
    x_shift = 0
    y_shift = 0
    pre_gen_obs = obs
    pre_gen_obs[:,0]*=-1
    robotx_g = -mapmaker.roboty
    roboty_g = mapmaker.robotx
    points, = ax.plot(robotx_g, roboty_g,"xr")  # empty points

    generator = GenerateMap(points, file, px_conv, bw_thresh,
                obstacleSize,theta,x_shift,y_shift,
                pre_gen_obs, robotx_g, roboty_g)
    plt.show()

    start = generator.start
    goal =  generator.goal
    obstacleList = generator.obs
    xmin = generator.xmin
    ymin = generator.ymin
    xmax = generator.xmax
    ymax = generator.ymax

    smooth = RRTSmooth()
    rrt = RRTSearch(start, goal, [xmin, xmax, ymin, ymax],
              obstacleList, expandDis, goalSampleRate, maxIter)
    path = rrt.Planning(animation=show_animation)

    # Path smoothing

    smoothedPath = smooth.PathSmoothing(path, maxIter, obstacleList)
    waypoints = smooth.GenerateWaypoints(smoothedPath,z,waypoint_thresh)
    print waypoints.tolist()
    idx = 0
    while idx < len(waypoints):
        wp_cur =  waypoints[idx] 
        rospy.wait_for_service('/slammer/add_waypoint')
        try:
            success = mapmaker.new_waypoint(x=wp_cur[0],y=wp_cur[1],z=wp_cur[2],yaw=wp_cur[3])
            if success:
                print "waypoint added"
                idx +=1
        except rospy.ServiceException,e:
            print "service call add_waypoint failed: %s" %e

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

if __name__ == '__main__':
    main()
