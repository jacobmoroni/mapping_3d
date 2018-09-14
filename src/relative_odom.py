#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations
from nav_msgs.msg import Odometry

class relativeOdom():
    def __init__(self):
        self.odomKeyframe = Odometry()
        self.relOdom = Odometry()
        self.kfLoc = (0,0,0)
        self.kfQuat = np.array([0,0,0,1])
        self.odomLoc = (0,0,0)
        self.odomQuat = np.array([0,0,0,1])
        self.relOdomLoc = (0,0,0)
        self.relOdomQuat = np.array([0,0,0,1])
        self.newKeyframe = False
        #set up publishers and subscribers
        self.listener = tf.TransformListener()
        self.odom_sub_ = rospy.Subscriber('/rtabmap/odom', Odometry, self.odometry_callback, queue_size = 5)
        self.odom_pub_ = rospy.Publisher('/relative_odom', Odometry, queue_size = 5, latch = True)

    def update(self):
        pass

    def odometry_callback(self,msg):
        self.get_transform()
        self.broadcast_transform('keyframe')


        if self.newKeyframe == True:
            self.kfLoc = self.odomLoc
            self.kfQuat = self.odomQuat
            self.newKeyframe = False
       
        self.relOdom.header = msg.header
        
        self.relOdom.pose.pose.position.x = self.relOdomLoc[0] 
        self.relOdom.pose.pose.position.y = self.relOdomLoc[1]
        self.relOdom.pose.pose.position.z = self.relOdomLoc[2]

        self.relOdom.pose.pose.orientation.w = self.relOdomQuat[0]
        self.relOdom.pose.pose.orientation.x = self.relOdomQuat[1]
        self.relOdom.pose.pose.orientation.y = self.relOdomQuat[2]
        self.relOdom.pose.pose.orientation.z = self.relOdomQuat[3]

        self.relOdom.pose.covariance = msg.pose.covariance
        self.relOdom.twist = msg.twist
        
        self.set_new_keyframe(self.relOdom)
        self.odom_pub_.publish(self.relOdom)

    def broadcast_transform(self, name):
        br = tf.TransformBroadcaster()
        br.sendTransform(self.kfLoc,self.kfQuat,rospy.Time.now(),name,'/odom')
        try:
            (self.relOdomLoc,self.relOdomQuat) = self.listener.lookupTransform(
                '/keyframe','/camera_link',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("cannot get tf transfrom")
            return

    def get_transform(self):
        """
        Get Robot Location from ROS
        """
        try:
            (trans, rot) = self.listener.lookupTransform('/odom', '/camera_link', rospy.Time(0))
            self.got_tf = True
            self.odomLoc = trans
            self.odomQuat = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("cannot get tf transform")
            self.got_tf = False
            return

    def set_new_keyframe(self,msg): 
        norm = np.sqrt(msg.pose.pose.position.x**2 + msg.pose.pose.position.y**2 + msg.pose.pose.position.z**2)
        quatdiff = self.kfQuat * tf.transformations.quaternion_inverse(self.relOdomQuat)
        (_,_,yaw_diff) = tf.transformations.euler_from_quaternion(quatdiff)
        if norm > 1 or abs(yaw_diff) > 0.35:
            self.newKeyframe = True

def main():
    rospy.init_node('RelativeOdom', anonymous=True)
    relativeodom = relativeOdom()

    while not rospy.is_shutdown():
        rospy.spin
if __name__ == '__main__':
    main()
