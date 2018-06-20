#!/usr/bin/env python

import numpy as np

import rospy, tf
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

class Restter():

    def __init__(self):
        self.counter = 0

        self.prev_time = rospy.get_time()
        self.t = 0
        # Set Up Publishers and Subscribers
        self.odom_sub_ = rospy.Subscriber('/rtabmap/odom', Odometry, self.odometryCallback, queue_size=5)
        # self.command_pub_ = rospy.Publisher('v_command', Command, queue_size=5, latch=True)

    def update(self):
        pass

    def odometryCallback(self, msg):
        self.t = rospy.get_time()
        cov = msg.twist.covariance[0]
        if cov > 9000:
            self.counter+=1
        if cov < 9999:
            self.counter = self.counter/2

        if self.counter > 10:
            self.counter = 5
            # rospy.wait_for_service('mavros/mission/pull')
            try:
                pull = rospy.ServiceProxy('/rtabmap/reset_odom', Empty)
                resp = pull()
                print "Odometry Forced Reset"

            except rospy.ServiceException as e:
                rospy.logerr("odometery reset failed: %s", e)

def main():
    rospy.init_node('odom_resetter',anonymous=True)

    resetter = Restter()

    while not rospy.is_shutdown():
        try:
            resetter.update()

        except rospy.ROSInterruptException:
            print "exiting..."
            return

if __name__ == '__main__':
    main()
