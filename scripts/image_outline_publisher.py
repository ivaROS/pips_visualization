#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Vector3
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
from image_geometry import PinholeCameraModel
from egocylindrical.msg import EgoCylinderPoints


def displayMat(name, image):
    minv, maxv, minloc, maxloc = cv2.minMaxLoc(image)

    copy = image.copy()
    copy[np.where(copy != copy)] = 0

    viz = copy * 1.0 / maxv

    cv2.imshow(name, viz)
    cv2.waitKey(1)




class OutlinePublisher():


    def imageCB(self, image_msg):
        rospy.logdebug("Hallucinated image callback")

        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

            real_image = np.zeros_like(image, dtype=np.uint8)
            real_image[np.where(image == image)] = 255

            contour_ret, contours, hierarchy = cv2.findContours(real_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #probably use RETR_LIST since we don't need hierarchy. APPROX_SIMPLE: looks good, except for curves
            #Note: Contour points use (row, col) notation

            contour_values = np.zeros_like(image)

            for contour in contours:

                for uv in contour:
                    if isinstance(uv, np.ndarray):
                        uv = uv.flatten()  # Note: use ravel() for view

                    coord = (uv[1], uv[0])
                    depth = image[coord]

                    if depth == depth:
                        contour_values[coord] = depth
                    else:
                        rospy.logwarn("Nan value in contour?!?!")

            rospy.logdebug("Added contour points")

            if self.debug:
                displayMat("image", image)
                displayMat("mask", real_image)

                contour_im = np.zeros_like(image, dtype=np.uint8)
                cv2.drawContours(contour_im, contours, -1, (255), 1)
                displayMat("contours", contour_im)
                displayMat("contour values", contour_values)

            outline_msg = self.bridge.cv2_to_imgmsg(contour_values, encoding="passthrough")
            outline_msg.header = image_msg.header

            self.im_pub.publish(outline_msg)


        except CvBridgeError as e:
            rospy.loginfo(e)

        #displayMat("Final image", img)



    def __init__(self):
        rospy.init_node('outline_publisher')

        self.debug = False

        self.image_out = 'image_out'

        self.image_topic = 'image_in'

        rospy.loginfo("Image Outline publisher starting up")

        self.im_pub = rospy.Publisher(self.image_out, Image, queue_size=5)

        self.im_sub = rospy.Subscriber(self.image_topic, Image, self.imageCB)

        self.bridge = CvBridge()

        rospy.loginfo("Image Outline Publisher ready")

        rospy.spin()


if __name__ == '__main__':
    try:
        OutlinePublisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
