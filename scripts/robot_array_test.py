#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from std_msgs.msg import Header

class Prototype():
  def __init__(self):
    rospy.init_node('robot_array_publisher')
    #rospy.on_shutdown(self.shutdown)
    
    self.pubArray = rospy.Publisher('poses', PoseArray, queue_size=10)
    
    self.frame_id = 'base_footprint'
    self.numRobots = 5
    freq = .25
    r = rospy.Rate(freq)
    


    rospy.loginfo("Robot Array Publisher Starting")

    header = Header(frame_id = self.frame_id)

    origin = PoseArray(header=header)
    #origin.header.frame_id = self.frame_id

    while not rospy.is_shutdown():
      poseMsg = PoseArray()
      poseMsg.header.frame_id = self.frame_id
      

      
      for i in range(0,self.numRobots):
        pose = Pose()
        
        pose.position.x = 1
        pose.position.y = random.uniform(-1,1)

        pose.orientation.w = 1
        
        #self.pubSingle.publish(path)
        poseMsg.poses.append(pose)

      self.pubArray.publish(poseMsg)
      r.sleep()

if __name__ == '__main__':
  try:
    Prototype()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
