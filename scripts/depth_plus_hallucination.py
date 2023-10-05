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
    for i in range(0, len(data.paths)):
      for j in range(0, len(data.paths[i].poses)):
        poseMsg = PoseArray(header = data.header)
        pose = data.paths[i].poses[j].pose
        poseMsg.poses.append(pose)
        
        try:
          resp = self.depthService(pose)
          
          self.imgPub.publish(resp.image)
          self.pubArray.publish(poseMsg)
        except rospy.ServiceException as exc:
          print("Service did not process request: " + str(exc))
        #self.r.sleep()

    self.pathSub.unregister()
    
    
    

  def __init__(self):
    rospy.init_node('depth_projection_tester')
    #rospy.on_shutdown(self.shutdown)
    
    self.serviceName = 'generate_depth_image';
    self.topicName = 'generated_depth_image';
    
    self.pubArray = rospy.Publisher('poses', PoseArray, queue_size=1000)
    self.pathSub = rospy.Subscriber('/GenAndTest/tested_paths', PathArray, self.pathCallback)
    self.imgPub = rospy.Publisher(self.topicName, Image, queue_size=1000)
    
    freq = 10
    self.r = rospy.Rate(freq)
    
    rospy.loginfo("Depth Projection Tester starting up")
    
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service(self.serviceName)
    self.depthService = rospy.ServiceProxy(self.serviceName, GenerateDepthImage)
    
    rospy.loginfo("Service found, commencing loop..")
    
    rospy.spin()
  


if __name__ == '__main__':
  try:
    Prototype()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
