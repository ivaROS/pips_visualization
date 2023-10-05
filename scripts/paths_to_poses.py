#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header
from pips.srv import GenerateDepthImage
from sensor_msgs.msg import Image

    
class Prototype():


  def pathCallback(self, data):
    if isinstance(data, PathArray):
      paths = data.paths
    elif isinstance(data, Path):
      paths = [data]

    poseMsg = PoseArray(header=data.header)

    for path in paths:

      if self.only_last:
        poses = [path.poses[-1]]
      else:
        poses = path.poses

      for pose in poses:
        poseMsg.poses.append(pose.pose)

    self.pubArray.publish(poseMsg)
    
    

  def __init__(self):
    rospy.init_node('paths_to_poses_publisher')

    self.pubArray = rospy.Publisher('poses', PoseArray, queue_size=1000)
    self.pathsSub = rospy.Subscriber('paths', PathArray, self.pathCallback)
    self.pathSub = rospy.Subscriber('path', Path, self.pathCallback)


    #freq = 10
    #self.r = rospy.Rate(freq)

    self.only_last = rospy.get_param('~only_last',False)

    rospy.loginfo("Paths to poses publisher starting up")
    

    rospy.spin()
  


if __name__ == '__main__':
  try:
    Prototype()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
