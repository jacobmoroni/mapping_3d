#!/usr/bin/env python
import numpy as np
import rospy
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class ObstacleDetect():
    def __init__(self):
        self.front_thresh = rospy.get_param('slammer/planner/forward_threshold',1.0)
        self.side_thresh = rospy.get_param('slammer/planner/side_threshold',0.6)
        self.obs_thresh = self.side_thresh
        self.replan_flag_sent = False
        
        self.nearest_obstacle_pub_ = rospy.Publisher('nearest_obstacle', Float32, queue_size=5, latch=True)
        self.laserscan_sub_ = rospy.Subscriber('/slammer/slammer/scan', LaserScan,
                self.laserscanCallback , queue_size=5) 
        self.new_waypoint = rospy.ServiceProxy('/slammer/add_waypoint', AddWaypoint)
        self.rm_waypoints = rospy.ServiceProxy('/slammer/remove_waypoint', RemoveWaypoint)
    
    def laserscanCallback(self,msg):
        min_idx = np.argmin(msg.ranges)
        obs_angle = msg.angle_min + min_idx*msg.angle_increment
        obstacle_angle = Float32()
        obstacle_angle.data = obs_angle
        self.nearest_obstacle_pub_.publish(obstacle_angle)
        # obs_angle_deg = obs_angle*180/np.pi
        # print obs_angle_deg
        # print min(msg.ranges)
        if abs(obs_angle)<=np.pi/6:
            self.obs_thresh = self.front_thresh
        else:
            self.obs_thresh = self.side_thresh
        if min(msg.ranges) <= self.obs_thresh and not self.replan_flag_sent:
            rospy.wait_for_service('/slammer/remove_waypoint')
            try:
                success = self.rm_waypoints(front = True)
                if success:
                    print "obstacle_detected: replan now"
                    self.replan_flag_sent = True
            except rospy.ServiceException,e:
                print "service call add_waypoint failed: %s" %e

        elif min(msg.ranges) > self.obs_thresh:
            self.replan_flag_sent = False
def main():
    rospy.init_node('obstacle_detection', anonymous=True)
    obstacledetect = ObstacleDetect()
    
    while not rospy.is_shutdown():
        rospy.spin
if __name__ == '__main__':
    main()
