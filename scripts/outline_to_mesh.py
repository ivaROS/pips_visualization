#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Vector3
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2


class OutlineToMeshPublisher():


    def pointsCB(self, points_msg):
        rospy.logdebug("Point cloud callback")

        marker = Marker()
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.header = points_msg.header

        marker.scale = Vector3(x=1,y=1,z=1)

        marker.ns="swept volume"

        marker.color = ColorRGBA(a=.25, b = 1)
        #Note: The following don't need to be changed
        # marker.pose
        # marker.id=0

        points = marker.points

        origin = [0,0,0]

        origin = Point(*origin)

        def pointToMsg(point):
            pnt = Point(*point)
            return pnt

        rospy.logdebug("Reading cloud...")
        cloud = pcl2.read_points(points_msg, skip_nans=True, field_names=("x", "y", "z"))
        rospy.logdebug("Done reading cloud...")

        first_pnt = None
        prev_pnt = None

        if points_msg.width > 1:
            for pnt in cloud:
                new_pnt = pointToMsg(pnt)

                if first_pnt is None:
                    first_pnt = new_pnt
                else:
                    points += [prev_pnt, origin, new_pnt]
                prev_pnt = new_pnt

            points += [prev_pnt, first_pnt, origin]

        rospy.logdebug("Added outline points")

        self.marker_pub.publish(marker)
        rospy.logdebug("Publishing marker")




    def __init__(self):
        rospy.init_node('outline_to_mesh_publisher')

        self.debug = False

        self.visualization_topic = 'swept_volume'

        self.cloud_topic = 'points'

        rospy.loginfo("Swept volume publisher starting up")

        self.marker_pub = rospy.Publisher(self.visualization_topic, Marker, queue_size=5)

        self.point_sub = rospy.Subscriber(self.cloud_topic, PointCloud2, self.pointsCB)


        rospy.loginfo("Hallucinated Robot Swept Volume Publisher ready")

        rospy.spin()


if __name__ == '__main__':
    try:
        OutlineToMeshPublisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
