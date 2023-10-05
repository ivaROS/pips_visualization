#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from pips.srv import GenerateDepthImage
from sensor_msgs.msg import Image
import cv2
import numpy as np
    
class Prototype():

  #def imageCallback(self, data):
  #  self.depthIm = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    
  def dpathCallback(self, data):

    pathMsg = Path(header = data.header)
    
    path = data.paths[self.path_num]

    pathMsg.poses.append(path.poses)

    self.dpubArray.publish(path)


  def tpathCallback(self, data):

    poseMsg = PoseArray(header = data.header)
    
    path = data.paths[self.path_num]

    for i in range(0, len(path.poses)):
      pose = path.poses[i].pose
      poseMsg.poses.append(pose)

    self.tpubArray.publish(poseMsg)


  def __init__(self):
    rospy.init_node('near_identity_viz')
    #rospy.on_shutdown(self.shutdown)
    
    
    
    rospy.loginfo("Near identity visualization starting up")
    
    self.path_num = 8
    
    
    self.dpubArray = rospy.Publisher('desired_poses', Path, queue_size=5)
    self.tpubArray = rospy.Publisher('tested_poses', PoseArray, queue_size=5)
    
        
    self.desSub = rospy.Subscriber('GenAndTest/desired_paths', PathArray, self.dpathCallback)
    self.tenSub = rospy.Subscriber('GenAndTest/tested_paths', PathArray, self.tpathCallback)
    

    
    rospy.loginfo("")

    rospy.spin()
    
  


if __name__ == '__main__':
  try:
    Prototype()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
