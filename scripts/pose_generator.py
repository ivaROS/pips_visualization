#!/usr/bin/env python

import math
import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Quaternion
import tf_conversions
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from pips.srv import GenerateDepthImage
from sensor_msgs.msg import Image
import cv2
import numpy as np


def RadialPoseGenerator(r, divs):
  yaw = 0
  pitch = 0;
  roll = 0

  inc = 2*math.pi / divs

  while True:
    pose = Pose()

    pose.position.x = r * math.cos(yaw)
    pose.position.y = r * math.sin(yaw)
    pose.position.z = 0

    pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw))

    yaw+=inc

    yield pose

def ForwardPoseGenerator(x, y_range):
    while True:
      pose = Pose()

      pose.position.x = x
      pose.position.y = -.5 #random.uniform(-y_range, y_range)
      pose.position.z = 0

      pose.orientation.w = 1

      yield pose

def TopLeftGenerator():
    while True:
      pose = Pose()

      pose.position.x = -.3
      pose.position.y = 0
      pose.position.z = .3

      pose.orientation.w = 1

      yield pose

def getQuat(yaw):
    return Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))


class Prototype():






  def __init__(self):
    rospy.init_node('hallucinated_pose_generator')
    #rospy.on_shutdown(self.shutdown)

    self.pose_topic = "visualization/poses"

    self.pose_pub = rospy.Publisher(self.pose_topic, PoseArray, queue_size=5)

    rospy.loginfo("Pose Generator starting up")

    freq = 1
    r = rospy.Rate(freq)

    base_frame_id = rospy.get_param("~base_frame_id", "base_footprint")

    pose_generator = RadialPoseGenerator(1,30)
    pose_generator = ForwardPoseGenerator(2,1)
    #pose_generator = TopLeftGenerator()
    
    while not rospy.is_shutdown():
      pose_array = PoseArray()
      pose_array.header.frame_id=base_frame_id
      pose_array.header.stamp = rospy.Time.now()

      #for i in xrange(random.randint(1,1)):
      #  pose_array.poses.append(next(pose_generator))

      for y in [0]: # np.arange(-.5,.6,.5):
        pose = Pose()

        pose.position.x = 1.2
        pose.position.y = y
        pose.position.z = 0

        yaw = random.uniform(-math.pi/3,math.pi/3)

        pose.orientation = getQuat(yaw=yaw)
        pose_array.poses.append(pose)

      self.pose_pub.publish(pose_array)

      r.sleep()
  


if __name__ == '__main__':
  try:
    Prototype()
  except rospy.ROSInterruptException:
    rospy.loginfo("Closing Pose Generator")
