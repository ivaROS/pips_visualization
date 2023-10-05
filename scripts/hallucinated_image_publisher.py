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

def displayMat(name, image):
    minv, maxv, minloc, maxloc = cv2.minMaxLoc(image)

    copy = image.copy()
    copy[np.where(copy != copy)] = 0

    viz = copy * 1.0 / maxv

    cv2.imshow(name, viz)
    cv2.waitKey(1)


class Prototype():

    def poseCB(self, poses):
        rospy.logdebug("Pose callback")
        img = None

        header = Header()

        for pose in poses.poses:

            try:
                resp = self.depthService(pose)
                header = resp.image.header

                try:
                    cv_image = self.bridge.imgmsg_to_cv2(resp.image, desired_encoding="passthrough")

                    copy = cv_image.copy()

                    rows, cols = copy.shape
                    if rows > cols:
                        copy = cv2.transpose(copy)

                    if self.debug:
                        displayMat("copy", copy)

                    if img is None:
                        img = copy
                    else:
                        #with np.errstate(invalid='ignore'):
                        img = np.fmax(img, copy)

                    if self.debug:
                        displayMat("current result", img)
                except CvBridgeError as e:
                    rospy.loginfo(e)

            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            except Exception as e:
                print("Other exception: " + str(e))

        if img is not None:
            if self.debug:
                displayMat("Final image", img)

            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
            img_msg.header = header
            self.imgPub.publish(img_msg)

    def __init__(self):
        rospy.init_node('hallucinated_robot_publisher')
        # rospy.on_shutdown(self.shutdown)

        #TODO: Use ros parameters to set service and topic names, allowing launch files to start up a desired config

        self.debug = False

        self.serviceName = rospy.get_param('~service_name', '/move_base/EgocylindricalPipsDWAPlannerROS/egocylindrical_image_cc_wrapper/egocylindrical_image_collision_checker/generate_depth_image')
        self.generated_image_topic = 'generated_image'
        self.pose_topic = "poses"

        rospy.loginfo("Hallucinated Robot Depth Image Generator starting up")

        rospy.loginfo("Waiting for service... [" + str(self.serviceName) + "]")
        rospy.wait_for_service(self.serviceName)
        self.depthService = rospy.ServiceProxy(self.serviceName, GenerateDepthImage)
        rospy.loginfo("Service found!")

        self.imgPub = rospy.Publisher(self.generated_image_topic, Image, queue_size=5)

        self.poseSub = rospy.Subscriber(self.pose_topic, PoseArray, self.poseCB)

        self.bridge = CvBridge()

        rospy.loginfo("Hallucinated Robot Depth Image Generator ready")

        rospy.spin()


if __name__ == '__main__':
    try:
        Prototype()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
