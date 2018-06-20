#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from tqdm import tqdm
# from pyquat import Quaternion
import rosbag
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu, Range, CompressedImage, CameraInfo
# from aruco_localization.msg import MarkerMeasurementArray
from rosflight_msgs.msg import Attitude
import scipy.signal
from pdb import set_trace


def calculate_velocity_from_position(t, position, orientation):
    # Calculate body-fixed velocity by differentiating position and rotating
    # into the body frame
    b, a = scipy.signal.butter(8, 0.03)  # Create a Butterworth Filter
    # differentiate Position
    delta_x = np.diff(position, axis=0)
    delta_t = np.diff(t)
    unfiltered_inertial_velocity = np.vstack((np.zeros((1, 3)), delta_x / delta_t[:, None]))
    # Filter
    v_inertial = scipy.signal.filtfilt(b, a, unfiltered_inertial_velocity, axis=0)
    # Rotate into Body Frame
    vel_data = []
    for i in range(len(t)):
        q_I_b = Quaternion(orientation[i, :, None])
        vel_data.append(q_I_b.rot(v_inertial[i, None].T).T)

    vel_data = np.array(vel_data).squeeze()
    return vel_data


def load_data(filename):
    pass
    # print "loading rosbag", filename
    # # First, load IMU data
    # bag = rosbag.Bag(filename)
    # timestamp_data = []
    # timestamp_data2 = []
    #
    # for topic, msg, t in tqdm(bag.read_messages(topics=['camera/color/image_rect_color/compressed',
    #                                                     'camera/aligned_depth_to_color/image_raw/compressedDepth',
    #                                                     '/tf',
    #                                                     '/tf_static',
    #                                                     'camera/color/camera_info',
    #                                                     'camera/depth/camera_info',
    #                                                     '/sonar',
    #                                                     '/encoder'
    #                                                     ])):


    #     if topic == 'camera/color/image_rect_color/compressed':
    #         timestamp_datapoint = [msg.header.stamp.secs, msg.header.stamp.nsecs, msg.header.seq]#, msg.header.frame_id]
    #         timestamp_data.append(timestamp_datapoint)
    #
    #     if topic == 'camera/aligned_depth_to_color/image_raw/compressedDepth':
    #         timestamp_datapoint2 = [msg.header.stamp.secs, msg.header.stamp.nsecs, msg.header.seq]#, msg.header.frame_id]
    #         timestamp_data2.append(timestamp_datapoint2)
    #
    #     if topic == 'camera/color/camera_info':
    #         timestamp_datapoint2 = [msg.header.stamp.secs, msg.header.stamp.nsecs, msg.header.seq]#, msg.header.frame_id]
    #         timestamp_data2.append(timestamp_datapoint2)
    #
    # timestamp_data = np.array(timestamp_data)
    # timestamp_data2 = np.array(timestamp_data2)
    #
    # # vel_data = np.hstack((timestamp_data))
    # # set_trace()
    # return timestamp_data, timestamp_data2


def bagwriter(filename, outbag):

    bag = rosbag.Bag(filename)
    for topic, msg, t in tqdm(bag.read_messages(topics=['camera/color/image_rect_color/compressed',
                                                        'camera/aligned_depth_to_color/image_raw/compressedDepth',
                                                        '/tf',
                                                        '/tf_static',
                                                        'camera/color/camera_info',
                                                        'camera/depth/camera_info',
                                                        '/sonar',
                                                        '/encoder'
                                                        ])):

        if topic == 'camera/color/image_rect_color/compressed':
            outbag.write(topic, msg, msg.header.stamp)
            # print msg.header.stamp

        if topic == 'camera/aligned_depth_to_color/image_raw/compressedDepth':
            outbag.write(topic, msg, msg.header.stamp)

        if topic == '/tf':
            outbag.write(topic, msg, msg.transforms[0].header.stamp)

        if topic == '/tf_static':
            outbag.write(topic, msg, msg.transforms[0].header.stamp)

        if topic == 'camera/color/camera_info':
            outbag.write(topic, msg, msg.header.stamp)

        if topic == 'camera/depth/camera_info':
            outbag.write(topic, msg, msg.header.stamp)

        if topic == '/sonar':
            outbag.write(topic, msg, msg.header.stamp)

        if topic == '/encoder':
            outbag.write(topic, msg, msg.header.stamp)






        # tf_stat = tf_data
        # for i in range(0,11):
        #     tf_stat.transforms[i].header.stamp.secs=data[0,0]
        #     tf_stat.transforms[i].header.stamp.nsecs=data[0,1]

        # print velocities.header.stamp
        # print t
    #     if cam_color_info.header.stamp < t and i<len(data)-1:
    #         i+=1
    #     if cam_depth_info.header.stamp < t and j<len(data2)-1:
    #         j+=1
    #     #     # print 'should be working'
    #     #     velocities.header.seq = data[i,2]
    #     #     velocities.vector.x = data[i,3]
    #     #     velocities.vector.y = data[i,4]
    #     #     velocities.vector.z = data[i,5]
    #     #     outbag.write('/velocities', velocities, velocities.header.stamp)
    #
    # tf_stat = TFMessage()
    # tf_stat = tf_data
    # for num in range(0,11):
    #     tf_stat.transforms[num].header.stamp.secs=data[0,0]
    #     tf_stat.transforms[num].header.stamp.nsecs=data[0,1]
    #     print data[0,0]
    #     print data[0,1]
    # # print tf_stat
    # outbag.write('/tf_static', tf_stat, tf_stat.transforms[0].header.stamp)

if __name__ == '__main__':
    outbag = rosbag.Bag('/home/jacob/6_5_t1_new.bag', 'w')
    inbag = '/home/jacob/6_5_t1.bag'
    # refbag = '/home/jacob/bags/bag_test.bag'
    # [tf_data, cinfo_data, dinfo_data] = load_ref_data(refbag)
    # [data,data2] = load_data(inbag)
    # bagwriter(inbag, outbag, data, data2, tf_data, cinfo_data, dinfo_data)
    bagwriter(inbag, outbag)

    print "done"
    outbag.close()
