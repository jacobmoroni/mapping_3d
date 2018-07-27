#!/usr/bin/env python

import numpy as np

import rospy, tf

from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
from std_msgs.msg import Int16

class WaypointManager():

    def __init__(self):

        # get parameters
        try:
            self.waypoint_list = rospy.get_param('~waypoints')
        except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')


        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('~threshold', 5)
        self.cyclical_path = rospy.get_param('~cycle', True)
        self.prev_time = rospy.Time.now()

        # set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.removeWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.end_waypoint_pub_ = rospy.Publisher('last_waypoint', Int16 , queue_size = 5, latch = True)

        self.current_waypoint_index = 0
        self.reset_waypoints = False
        self.current_yaw = 0.0
        
        self.new_waypoint = rospy.ServiceProxy('/slammer/add_waypoint', AddWaypoint)
        command_msg = Command()
        current_waypoint = np.array(self.waypoint_list[0])

        command_msg.x = current_waypoint[0]
        command_msg.y = current_waypoint[1]
        command_msg.F = current_waypoint[2]
        if len(current_waypoint) > 3:
            command_msg.z = current_waypoint[3]
        else:
            next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
            delta = next_point - current_waypoint
            command_msg.z = np.atan2(delta[1], delta[0])
        command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_pub_.publish(command_msg)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def addWaypointCallback(self,req):
        print("addwaypoints")
        print req
        new_waypoint = [req.x,req.y,req.z,req.yaw]
        self.waypoint_list.append(new_waypoint)
        length = 4
        return length

    def removeWaypointCallback(self,req):
        print("remove Waypoints")
        self.waypoint_list = self.waypoint_list[0:self.current_waypoint_index]
        self.current_waypoint_index -= 1
        self.reset_waypoints = True
        length = 1
        return length

    def setWaypointsFromFile(req):
        print("set Waypoints from File")

    def odometryCallback(self, msg):
        # Get error between waypoint and current state
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z])

        error = np.linalg.norm(current_position - current_waypoint[0:3])
        
        if self.reset_waypoints:
            command_msg = Command()
            command_msg.x = current_position[0]
            command_msg.y = current_position[1]
            command_msg.F = current_position[2]
            command_msg.z = self.current_yaw 
            command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
            self.waypoint_pub_.publish(command_msg)
            
            rospy.wait_for_service('/slammer/add_waypoint')
            try:
                success = self.new_waypoint(x=current_position[0],y=current_position[1],
                                                z=current_position[2],yaw=self.current_yaw)
                if success:
                    print "waypoint added"
                    self.current_waypoint_index +=1
            except rospy.ServiceException,e:
                print "service call add_waypoint failed: %s" %e
            self.reset_waypoints = False
            
        if error < self.threshold:
            # Get new waypoint index
            self.current_waypoint_index += 1
            if self.cyclical_path:
                self.current_waypoint_index %= len(self.waypoint_list)
            else:
                if self.current_waypoint_index >= len(self.waypoint_list):
                    self.current_waypoint_index -=1
                    if not self.last_wp_published:
                        flag = Int16()
                        flag.data = 1
                        self.end_waypoint_pub_.publish(flag)
                    self.last_wp_published = True
                else:
                    self.last_wp_published = False
            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            command_msg = Command()
            command_msg.x = next_waypoint[0]
            command_msg.y = next_waypoint[1]
            command_msg.F = next_waypoint[2]
            if len(current_waypoint) > 3:
                command_msg.z = current_waypoint[3]
            else:
                next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
                delta = next_point - current_waypoint
                command_msg.z = np.atan2(delta[1], delta[0])
                self.current_yaw = command_msg.z
            command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
            self.waypoint_pub_.publish(command_msg)

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
