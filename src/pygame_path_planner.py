#!/usr/bin/env python
import numpy as np
import rospy, tf
import cv2
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from tf.transformations import euler_from_quaternion
import tf
from rrt_pygame import RRTSmooth, RRTSearch
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from std_msgs.msg import Int16
import pygame

pygame.init()
import math


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
        self.update_plot_timer = rospy.get_param('slammer/planner/update_plot_timer',50)
        self.pbuff = rospy.get_param('slammer/planner/pbuff',70)
        
        #initialize variables that are set from within the code
        self.reso = 0.05
        self.mapw = 0
        self.maph = 0
        self.originx = 0
        self.originy = 0
        self.map1d = []
        self.map2d = []
        self.robotx = None
        self.roboty = None
        self.obs = []
        self.current_waypoints = None
        self.goalx = 0.0
        self.goaly = 0.0
        self.current_waypoint_idx = -1
        self.clickx = 0
        self.clicky = 0
        self.xmin = -15
        self.xmax = 15
        self.ymin = -15
        self.ymax = 15
        self.screen_size = (int((self.xmax-self.xmin)/self.reso),int((self.ymax-self.ymin)/self.reso))
        self.screen = pygame.display.set_mode(self.screen_size)
        self.counter = 0

        #pygame colors
        self.black = (0,0,0)
        self.white = (255,255,255)
        self.red = (255,0,0)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.grey = (100,100,100)

        #initialize flags
        self.got_tf = False
        self.plot_now_flag = False
        self.bad_path = False
        self.wp_add_done = True
        self.choose_goal = False

        #set up publishers and subscribers
        self.listener = tf.TransformListener()
        self.last_waypoint_sub_ = rospy.Subscriber('/slammer/last_waypoint', Int16,
                self.replanCallback, queue_size = 1)
        self.check_path_sub_ = rospy.Subscriber('/slammer/check_path', Int16,
                self.checkPathCallback, queue_size = 1)
        self.gridmap_sub_ = rospy.Subscriber('rtabmap/grid_map', OccupancyGrid,
                self.gridmapCallback, queue_size=5)

        #set up services
        self.new_waypoint = rospy.ServiceProxy('/slammer/add_waypoint', AddWaypoint)
        self.rm_waypoints = rospy.ServiceProxy('/slammer/remove_waypoint', RemoveWaypoint)
   
        #set up pygame plotting callback timer
        rospy.Timer(rospy.Duration(0.01),self.pygamePlot,False)

    def textObjects(self, text, font):
        '''
        textObjects:
            generate text object for pygame 
        Input:
            text: text to be printed
            font: font to print in
        Returns: 
            textSurface and rectangle used to blit text onto screen
        '''
        textSurface = font.render(text, True, self.red)
        return textSurface, textSurface.get_rect()

    def messageDisplay(self,text,location):
        '''
        messageDisplay:
            display text message on pygame screen
        Input:
            text: text to be printed 
            location: [top,left] location of text rectangle
        '''
        largeText = pygame.font.Font('freesansbold.ttf',15)
        TextSurf, TextRect = self.textObjects(text, largeText)
        TextRect.top = location[0]
        TextRect.left = location[1]
        self.screen.blit(TextSurf, TextRect)

    def pygamePlot(self,tevent):
        '''
        pygamePlot:
            plots and displays planning GUI in pygame window
        input:
            tevent: generated from timer callback
        '''
        #set screen size based off of xmin and xmax from obstacles
        self.screen_size = (int((self.xmax-self.xmin)/self.reso+self.pbuff*2),
                int((self.ymax-self.ymin)/self.reso+self.pbuff*2))
        #check for keyboard and mouse inputs
        event = pygame.event.poll()
        #do this when the left mouse button is clicked
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self.clickx,self.clicky = event.pos
            self.plot_now_flag = True
        
        if self.choose_goal == True:
            #do this if ENTER/RETURN is pressed
            if event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
                self.choose_goal = False
        #this is where the plotting happens, runs every self.update_plot_timer ms 
        #   or when plot_now_flag is thrown
        if self.counter > self.update_plot_timer or self.plot_now_flag == True:
            self.goalx = ((self.reso*(self.clickx-self.pbuff))+self.xmin)
            self.goaly = -((self.reso*(self.clicky-self.pbuff))-self.ymax)
            self.getTransform()
            self.screen = pygame.display.set_mode(self.screen_size)
            self.screen.fill(self.white)
            # Plot Obstacles and obstacle buffers
            if len(self.obs) > 1:
                pg_obs = np.array([((-self.obs[:,0]-self.xmin)/self.reso+self.pbuff),
                        ((-self.obs[:,1]+self.ymax)/self.reso+self.pbuff)]).astype(int).T
                for ob in pg_obs:
                    pygame.draw.circle(self.screen,self.grey,(ob[0],ob[1]),
                            int(self.obstacleSize/self.reso),0)
                for ob in pg_obs:
                    pygame.draw.circle(self.screen,self.black,(ob[0],ob[1]),2,0)
            # Plot waypoints and connect them
            if self.current_waypoints is not None:
                pg_robotx = int((-self.roboty-self.xmin)/self.reso+self.pbuff) 
                pg_roboty = int((-self.robotx+self.ymax)/self.reso+self.pbuff)
                wp_line = [[pg_robotx,pg_roboty]]
                cwaypoints=self.current_waypoints[max(self.current_waypoint_idx,0):len(self.current_waypoints)]
                for wp in cwaypoints:
                    pygame.draw.circle(self.screen,self.blue,
                            (int((wp[1]-self.xmin)/self.reso+self.pbuff),
                                int((-wp[0]+self.ymax)/self.reso+self.pbuff)),3,0)
                    wp_line.append([int((wp[1]-self.xmin)/self.reso)+self.pbuff,
                                    int((-wp[0]+self.ymax)/self.reso)+self.pbuff])
                pygame.draw.lines(self.screen,(0,0,200),False,wp_line,2)
            #plot currently selected goal point
            pygame.draw.circle(self.screen,self.red,(self.clickx,self.clicky), 5,0)
            #plot current robot location
            if self.robotx is not None:
                pygame.draw.circle(self.screen,self.green,
                    (int((-self.roboty-self.xmin)/self.reso+self.pbuff),
                     int((-self.robotx+self.ymax)/self.reso+self.pbuff)),5,0)
            #display goal location
            self.messageDisplay("Goal Location: %f , %f" % (self.goalx,self.goaly),[10,10])
            #display "select goal" message
            if self.choose_goal == True:
                self.messageDisplay("Select new goal press enter when done",[25,10])
            self.counter = 0
            self.plot_now_flag = False
        
        pygame.display.flip()
        self.counter +=1

    def getTransform(self):
        """
        getTransform:
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

        # print "Current Agent Location: ","[", self.robotx, ",", self.roboty,"]"

    def gridmapCallback(self,msg):
        """
        gridmapCallback:
            parse callback message to generate obstacles
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
        # obs[:,0] *=-1
        self.obs = obs
        self.xmin = -max(obs[:,0])
        self.xmax = -min(obs[:,0])
        self.ymin = min(obs[:,1])
        self.ymax = max(obs[:,1])

    def replanCallback(self,msg):
        '''
        replanCallback:
            triggered by reaching last waypoint or bad path
            replans path to goal
            callback comes from waypoint planner
            1: last waypoint reached
            2: bad path/obstacle
        '''
        if msg.data == 1: 
            self.show_visualization = True
            fig = plt.figure(1)
            ax = fig.add_subplot(111)
            ax.set_title('Select Goal (Close when finished)')
        elif msg.data == 2: 
            self.show_visualization = False
        self.getTransform()
        while not self.got_tf: #make sure most recent transfrom is used
            self.getTransform()

        self.px_conv = self.reso
        theta = 0
        x_shift = 0
        y_shift = 0
        pre_gen_obs = self.obs.copy() #obstacles before going through GenerateMap
        pre_gen_obs[:,0]*=-1 #transform into correct frame
        sizes = np.ones([1,len(pre_gen_obs)])*self.obstacleSize
        obstacleList = np.append(pre_gen_obs.T,sizes,axis=0).T
        #rotate obstacle into graph frame to look the same as map
        robotx_g = -self.roboty #robot x location on pygame graph
        roboty_g = self.robotx #robot y location on pygame graph

        start = [robotx_g,roboty_g]
        if self.show_visualization:
            self.choose_goal = True
            while self.choose_goal == True:
                rospy.sleep(0.1)
            self.goal =  [self.goalx,self.goaly]
        else:
            #check to see if a goal has been set
            if not hasattr(self, 'goal'):
                self.goal = start
        
        smooth = RRTSmooth()
        #generate rrt path to end
        rrt = RRTSearch(start, self.goal, [self.xmin, self.xmax, self.ymin, self.ymax], obstacleList,
                self.show_obs_size, self.expandDis, self.goalSampleRate, self.maxIter)
        path = rrt.Planning(self.show_visualization, animation=self.show_animation)

        # Path smoothing
        smoothedPath = smooth.PathSmoothing(path, self.maxIter, obstacleList)
        waypoints = smooth.GenerateWaypoints(smoothedPath,self.z,self.waypointThresh)
        self.current_waypoints = waypoints
        self.current_waypoint_idx = -1
        print "Adding %d New Waypoints" % (len(waypoints))
        idx = 0 #index of new waypoints to be added
        self.wp_add_done = False
        while idx < len(waypoints):
            wp_cur =  waypoints[idx]
            rospy.wait_for_service('/slammer/add_waypoint')
            try:
                success = self.new_waypoint(x=wp_cur[0],y=wp_cur[1],z=wp_cur[2],yaw=wp_cur[3],
                        radius= self.thresh, difficulty = self.diff_factor)
                if success:
                    idx +=1
            except rospy.ServiceException,e:
                print "service call add_waypoint failed: %s" %e
        self.wp_add_done = True
    
    def checkPathCallback(self,msg):
        '''
        checkPathCallback:
            every time the robot gets to a new waypoint, 
            check to make sure path does not intersect obstacles
        '''
        print msg.data
        if self.current_waypoints is not None:
            cwaypoints = np.array([self.current_waypoints[:,1],self.current_waypoints[:,0]]).T
            
            check = RRTSmooth()
            
            sizes = np.ones([1,len(self.obs)])*self.obstacleSize/2
            obs_check = np.append(self.obs.T,sizes,axis=0).T
            obs = obs_check.copy()
            obs[:,0]*=-1
            for i in range(max(0,self.current_waypoint_idx),len(cwaypoints)-1):
                first = [cwaypoints[i,0],cwaypoints[i,1],i]
                second =  [cwaypoints[i+1,0],cwaypoints[i+1,1],i+1]
                if not check.LineCollisionCheck(first,second,obs):
                    self.bad_path = True
                else:
                    continue
            if self.bad_path and self.wp_add_done:

                rospy.wait_for_service('/slammer/remove_waypoint')
                try:
                    success = self.rm_waypoints(front = True)
                    if success:
                        print "obstacle_detected: replan now"
                        self.bad_path = False
                except rospy.ServiceException,e:
                    print "service call remove_waypoint failed: %s" %e
                
            self.current_waypoint_idx +=1
        else:
            pass

def main():
    rospy.init_node('MapMaker', anonymous=True)
    mapmaker = MapMaker()

    while not rospy.is_shutdown():
        rospy.spin
if __name__ == '__main__':
    main()
