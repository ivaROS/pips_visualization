#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from std_srvs.srv import Empty


    
class Prototype():


  def pathCallback(self, data):
    rospy.loginfo("Paths received. Pausing gazebo...")
    try:
      self.pauseService()
    except rospy.ServiceException as exc:
      rospy.logwarn("Service did not process request: " + str(exc))
    



  def __init__(self):
    rospy.init_node('plan_pause')

    rospy.loginfo("Pause on plan node starting up")
    
    self.serviceName = "/gazebo/pause_physics";
    
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service(self.serviceName)
    
    self.pauseService = rospy.ServiceProxy(self.serviceName, Empty)
    rospy.loginfo("Service found. Listening for paths...")

    self.pathSub = rospy.Subscriber('GenAndTest/tested_paths', PathArray, self.pathCallback)

    rospy.spin()
    
  


if __name__ == '__main__':
  try:
    Prototype()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
