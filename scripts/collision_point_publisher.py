#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from pips.srv import TestCollision
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import cv2
import numpy as np
import message_filters
import time

class Prototype():

    def poseCB(self, poses):
        rospy.logdebug("Pose callback: " + str(poses.header.stamp))

        num_clouds = 0

        if len(poses.poses) > 0:
            ## Issue: Getting the correct time stamp.
            # resp = self.depthService(poses.poses[0])
            # header = resp.image.header

            latest_time = self.points_cache.getLastestTime()
            if latest_time is None:
                latest_time = rospy.Time(0)

            for pose in poses.poses:
                try:
                    resp = self.testingService(pose)
                    if resp.collision.data:
                        num_clouds+=1
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))


            target_time = poses.header.stamp


            if self.points_cache.getLast() is not None:
                if num_clouds > 0:

                    ##Necessary to give GIL time to process message callbacks. Ideally, this would be replaced by something
                    ##less brittle
                    time.sleep(.1)
                    next_elem = self.points_cache.getElemAfterTime(latest_time + rospy.Duration(nsecs=1))
                    if next_elem is not None:
                        next_time = next_elem.header.stamp

                        print "latest: " + str(latest_time) + ", " + str(target_time)


                        point_clouds = self.points_cache.getInterval(next_time, next_time)



                        # while len(point_clouds) < num_clouds:
                        #     #rospy.sleep(.1)
                        #     time.sleep(.1)
                        #     point_clouds = self.points_cache.getInterval(target_time, target_time)

                        if len(point_clouds) > 0:

                            class PointAccumulator():
                                def __init__(self):
                                    self.size = 0;
                                    for cloud in point_clouds:
                                        self.size += cloud.width

                                    self.clouds = self.getNext()

                                def __len__(self):
                                    return self.size

                                def getNext(self):
                                    for cloud in point_clouds:
                                        gen = pcl2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
                                        for point in gen:
                                            yield point

                                def next(self):
                                    return next(self.clouds)

                                def __iter__(self):
                                    return self

                            all_points = PointAccumulator()
                            combined_cloud = pcl2.create_cloud_xyz32(point_clouds[0].header,all_points)

                            self.points_pub.publish(combined_cloud)
                else:
                    cloud_points = [[0,0,0]]

                    combined_cloud = pcl2.create_cloud_xyz32(header=Header(stamp=target_time, frame_id=self.points_cache.getLast().header.frame_id), points=cloud_points)

                    self.points_pub.publish(combined_cloud)


    def pointsCB(self, point_cloud):
        rospy.logdebug("Point cloud received: " + str(point_cloud.header.stamp))


    def __init__(self):
        rospy.init_node('collision_point_publisher')


        self.serviceName = rospy.get_param('~service_name', '/move_base/EgocylindricalPipsDWAPlannerROS/egocylindrical_image_cc_wrapper/CollisionChecker/test_collision')
        self.pose_topic = "poses"

        self.collision_point_topic = "collision_points"
        self.combined_collision_point_topic = "combined_collision_points"


        rospy.loginfo("Collision Point Publisher starting up")

        rospy.loginfo("Waiting for service... [" + str(self.serviceName) + "]")
        rospy.wait_for_service(self.serviceName)
        self.testingService = rospy.ServiceProxy(self.serviceName, TestCollision)
        rospy.loginfo("Service found!")

        self.points_sub = message_filters.Subscriber(self.collision_point_topic, PointCloud2)
        self.points_cache = message_filters.Cache(self.points_sub, 100)
        self.points_cache.registerCallback(self.pointsCB)

        self.points_pub = rospy.Publisher(self.combined_collision_point_topic, PointCloud2, queue_size=5)

        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseArray, self.poseCB)
        self.point_cloud_frame = None

        rospy.loginfo("Collision Point Publisher ready")

        rospy.spin()


if __name__ == '__main__':
    try:
        Prototype()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
