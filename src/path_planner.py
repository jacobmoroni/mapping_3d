#!/usr/bin/env python
import numpy as np
import rospy, tf
import cv2
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from tf.transformations import euler_from_quaternion
import tf
from rrt_planner import GenerateMap, RRTSmooth, RRTSearch
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from std_msgs.msg import Int16




class MapMaker():
    """
    Class for Generating map for RRT Planning
    """
    def __init__(self):
        #get params -- see yaml file for explanation of parameters
        self.thresh = rospy.get_param('slammer/waypoint_manager/threshold',.2)
        self.diff_factor = rospy.get_param('slammer/watpoint_manager/diff_factor',.2)

        self.unknown_as_obs = rospy.get_param('slammer/planner/unknown_as_obs',False)
        self.show_obs_size = rospy.get_param('slammer/planner/show_obs_size', False)
        self.show_animation = rospy.get_param('slammer/planner/show_animation', False)
        self.show_visualization = rospy.get_param('slammer/planner/show_visualization',True)
        self.file = rospy.get_param('slammer/planner/file',0)
        self.bw_thresh = rospy.get_param('slammer/planner/bw_thresh',10)
        self.obstacleSize = rospy.get_param('slammer/planner/obstacleSize',0.8)
        self.z =rospy.get_param('slammer/planner/waypoint_z', -1.3)
        self.waypointThresh = rospy.get_param('slammer/planner/waypointThresh',0.09)
        self.expandDis =rospy.get_param('slammer/planner/expandDis', 0.5)
        self.goalSampleRate = rospy.get_param('slammer/planner/goalSampleRate',20)
        self.maxIter = rospy.get_param('slammer/planner/maxIter',500)

        #initialize variables
        self.reso = 0
        self.mapw = 0
        self.maph = 0
        self.originx = 0
        self.originy = 0
        self.map1d = []
        self.map2d = []
        self.robotx = None
        self.roboty = None
        self.got_tf = False
        self.obs = []

        #set up publishers and subscribers
        self.listener = tf.TransformListener()
        self.last_waypoint_sub_ = rospy.Subscriber('/slammer/last_waypoint', Int16,
                self.replanCallback, queue_size = 1)
        self.gridmap_sub_ = rospy.Subscriber('rtabmap/grid_map', OccupancyGrid,
                self.gridmapCallback, queue_size=5)

        #set up services
        self.new_waypoint = rospy.ServiceProxy('/slammer/add_waypoint', AddWaypoint)

    def getTransform(self):
        """
        Get Robot Location from ROS
        """
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
            self.got_tf = True
            self.robotx = trans[0]
            self.roboty = trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("cannot get tf transform")
            self.got_tf = False
            return

        print "Current Agent Location: ","[", self.robotx, ",", self.roboty,"]"

    def gridmapCallback(self,msg):
        """
        reso:resolution of occupancy grid
        mapw:occupancy grid width
        maph:occupancy grid height
        originx:location of corner of occupancy grid to origin x
        originy:location of corner of occupancy grid to origin y
        map1d:values of map in a 1d vector
        map2d:2d values of map
        """
        self.reso = msg.info.resolution
        self.mapw = msg.info.width
        self.maph = msg.info.height
        self.originx = msg.info.origin.position.x
        self.originy = msg.info.origin.position.y
        self.map1d = np.array(msg.data)
        self.map2d = self.map1d.reshape(self.maph,self.mapw)

        #generate obstacles with or without unknown as obstacles
        if not self.unknown_as_obs:
            obs = np.where(self.map2d==100)
        else:
            obs = np.where(np.logical_or(self.map2d==100, self.map2d==-1))

        #transform obstacles to correct orientation
        obs = np.array(obs)
        obs  = obs.T*self.reso
        obs[:,0] += self.originy
        obs[:,1] += self.originx
        self.obs = obs

    def replanCallback(self,msg):
        if msg.data == 1: #flag comes from waypoint planner, 1 = triggered by reaching last waypoint
            self.show_visualization = True
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.set_title('Select Goal (Close when finished)')
        elif msg.data == 2: #flac comes from waypoint planner, 2 = triggered by obstacle
            self.show_visualization = False
        self.getTransform()
        while not self.got_tf: #make sure most recent transfrom is used
            self.getTransform()

        self.px_conv = self.reso
        theta = 0
        x_shift = 0
        y_shift = 0
        pre_gen_obs = self.obs #obstacles before going through GenerateMap
        pre_gen_obs[:,0]*=-1 #transform into correct frame

        #rotate obstacle into graph frame to look the same as map
        robotx_g = -self.roboty #robot x location on matplotlib graph
        roboty_g = self.robotx #robot y location on matplotlib graph

        #initialize start position for path planning
        if self.show_visualization:
            points, = ax.plot(robotx_g, roboty_g,"xr")  # empty points
        else:
            points = []

        #generate map for planning
        generator = GenerateMap(points, self.file, self.px_conv, self.bw_thresh, self.obstacleSize,
                    self.unknown_as_obs, self.show_obs_size, self.show_visualization,
                    theta, x_shift, y_shift, pre_gen_obs, robotx_g, roboty_g)
        if self.show_visualization:
            plt.show()
            start = generator.start
            self.goal =  generator.goal
        else:
            start = [robotx_g,roboty_g]
            if not hasattr(self, 'goal'):
                self.goal = start

        obstacleList = generator.obs
        xmin = generator.xmin
        ymin = generator.ymin
        xmax = generator.xmax
        ymax = generator.ymax

        smooth = RRTSmooth()
        #generate rrt path to end
        rrt = RRTSearch(start, self.goal, [xmin, xmax, ymin, ymax], obstacleList,
                self.show_obs_size, self.expandDis, self.goalSampleRate, self.maxIter)
        path = rrt.Planning(self.show_visualization, animation=self.show_animation)

        # Path smoothing
        smoothedPath = smooth.PathSmoothing(path, self.maxIter, obstacleList)
        waypoints = smooth.GenerateWaypoints(smoothedPath,self.z,self.waypointThresh)
        print "Adding Waypoints: ", waypoints.tolist()
        idx = 0 #index of new waypoints to be added
        while idx < len(waypoints):
            wp_cur =  waypoints[idx]
            rospy.wait_for_service('/slammer/add_waypoint')
            try:
                success = self.new_waypoint(x=wp_cur[0],y=wp_cur[1],z=wp_cur[2],yaw=wp_cur[3],
                        radius= self.thresh, difficulty = self.diff_factor)
                if success:
                    print "waypoint added"
                    idx +=1
            except rospy.ServiceException,e:
                print "service call add_waypoint failed: %s" %e

        # Draw final path
        if self.show_visualization:
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

def main():
    rospy.init_node('MapMaker', anonymous=True)
    mapmaker = MapMaker()

    while not rospy.is_shutdown():
        rospy.spin
if __name__ == '__main__':
    main()
