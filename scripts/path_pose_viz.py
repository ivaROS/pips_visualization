#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header



    
class Prototype():


  def pathCallback(self, data):

    posesMsg = PoseArray(header = data.header)

    path = data.paths[self.path_num]

    for i in range(0, len(path.poses)):
      pose = path.poses[i]     
      
      if(i % self.num_skip==0):
        posesMsg.poses.append(pose.pose)
      #else:
      odom = Odometry(header = data.header)
      odom.pose.pose = pose.pose
      self.posePub.publish(odom)

    self.dpubArray.publish(path)
    self.pubArray.publish(posesMsg)



  def __init__(self):
    rospy.init_node('path_pose_publisher')
    #rospy.on_shutdown(self.shutdown)
    

    rospy.loginfo("Path Pose Publisher starting up")

    
    self.pubArray = rospy.Publisher('hallucinated_poses', PoseArray, queue_size=5)
    self.posePub = rospy.Publisher('path_poses', Odometry, queue_size=100)
        
    self.pathSub = rospy.Subscriber('GenAndTest/tested_paths', PathArray, self.pathCallback)
    self.dpubArray = rospy.Publisher('chosen_path', Path, queue_size=5)
    
    self.num_skip = 8
    self.path_num = 6

    


    rospy.spin()
    
  


if __name__ == '__main__':
  try:
    Prototype()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
