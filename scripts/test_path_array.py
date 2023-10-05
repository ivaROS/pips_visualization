#!/usr/bin/env python

import rospy
import random
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

class Prototype():
  def __init__(self):
    rospy.init_node('PathArrayPublisher')
    #rospy.on_shutdown(self.shutdown)
    
    self.pubArray = rospy.Publisher('paths', PathArray, queue_size=10)
    self.pubSingle = rospy.Publisher('path', Path, queue_size=10)
    
    self.frame_id = 'base_link'
    self.numPaths = 10
    freq = 2
    r = rospy.Rate(freq)
    


    rospy.loginfo("Path Array Publisher Starting")

    header = Header(frame_id = self.frame_id)

    origin = PoseStamped(header=header)
    #origin.header.frame_id = self.frame_id

    while not rospy.is_shutdown():
      pathMsg = PathArray()
      pathMsg.header.frame_id = self.frame_id
      

      
      for i in range(0,self.numPaths):
        path = Path(header=header)
        #path.header.frame_id = self.frame_id
        
        pose = PoseStamped(header=header)
        #pose.header.frame_id = self.frame_id
        pose.pose.position.x = 1
        pose.pose.position.y = random.uniform(-1,1)
        
        path.poses.append(origin)
        path.poses.append(pose)
        
        self.pubSingle.publish(path)
        pathMsg.paths.append(path)

      self.pubArray.publish(pathMsg)
      r.sleep()

if __name__ == '__main__':
  try:
    Prototype()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
