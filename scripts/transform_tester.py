#!/usr/bin/env python

import rospy
import random
import sys, os, time
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from copy import deepcopy
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

# from pips_test import gazebo_driver

from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
import numpy as np
from std_srvs.srv import Empty

from gazebo_ros import gazebo_interface
import std_srvs.srv
import cv2


import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header



class TransformTester():

    def poseCB(self, poses):
        rospy.logdebug("Pose callback")

        pose_header = poses.header

        if self.depth_header is not None:
            depth_transform = self.getTransform(target_header=self.depth_header, source_header=pose_header)

            if depth_transform is not None:
                rospy.loginfo("Depth Transform:\n" + str(depth_transform))

                header = Header(frame_id=self.depth_header.frame_id, stamp=pose_header.stamp)
                pose_array = PoseArray(header=header)

                for pose in poses.poses:
                    pose_stamped = PoseStamped(pose=pose)
                    transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, depth_transform)
                    pose_array.poses.append(transformed_pose.pose)

                    pass

                pass
                self.depth_poses_pub.publish(pose_array)

            pass

        if self.ec_header is not None:
            ec_transform = self.getTransform(target_header=self.ec_header, source_header=pose_header)

            if ec_transform is not None:
                rospy.loginfo("EC Transform:\n" + str(ec_transform))

                header = Header(frame_id=self.ec_header.frame_id, stamp=pose_header.stamp)
                pose_array = PoseArray(header=header)

                for pose in poses.poses:
                    pose_stamped = PoseStamped(pose=pose)
                    transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, ec_transform)
                    pose_array.poses.append(transformed_pose.pose)

                    pass

                pass
                self.ec_poses_pub.publish(pose_array)

            pass

        if self.ec_header is not None and self.depth_header is not None:
            transform = self.getTransform(target_header=self.ec_header,source_header=self.depth_header)
            rospy.loginfo("Transform= " + str(transform))

    def depthCB(self, image):
        self.depth_header = image.header
        rospy.logdebug("Depth image callback")

    def ecCB(self, image):
        self.ec_header = image.header
        rospy.logdebug("EC image callback")


    def getTransform(self, target_header, source_header):
        try:
            transform = self.tfBuffer.lookup_transform_full(
                target_frame=target_header.frame_id,
                target_time=target_header.stamp,
                source_frame=source_header.frame_id,
                source_time=source_header.stamp,
                fixed_frame=self.fixed_frame_id,
                timeout=rospy.Duration(1.0)
            )

            return transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            rospy.logerr(e)
            return None


    def __init__(self):

        rospy.init_node('transform_tester')

        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer)
        self.poseSub = rospy.Subscriber("poses", PoseArray, self.poseCB)
        self.depth_sub = rospy.Subscriber("depth_im", Image, self.depthCB)
        self.ec_sub = rospy.Subscriber("ec_im", Image, self.ecCB)
        self.depth_poses_pub = rospy.Publisher('depth_poses', PoseArray, queue_size=1000)
        self.ec_poses_pub = rospy.Publisher('ec_poses', PoseArray, queue_size=1000)


        self.depth_header = None
        self.ec_header = None

        self.fixed_frame_id = 'odom'
        rospy.spin()


if __name__ == '__main__':
    try:
        a = TransformTester()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
