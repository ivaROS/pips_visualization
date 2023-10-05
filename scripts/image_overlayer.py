#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import message_filters
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


class ImageOverlayer():


    def mainImageCB(self, image_msg):
        rospy.logdebug("Hallucinated image callback")

        target_time = image_msg.header.stamp

        try:
            filler_imgs = self.filler_im_cache.getInterval(target_time, target_time)

            if len(filler_imgs) > 0:
                filler_img = self.bridge.imgmsg_to_cv2(filler_imgs[0], desired_encoding="passthrough").copy()

                main_img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough").copy()

                mask_idx = np.where((main_img != main_img) | (main_img == 0))
                main_img[mask_idx] = filler_img[mask_idx]

                combined_img = self.bridge.cv2_to_imgmsg(main_img, encoding="passthrough")
                combined_img.header = image_msg.header
                self.imgPub.publish(combined_img)
            else:
                rospy.logwarn("No filler image found for time " + str(target_time))

        except CvBridgeError as e:
            rospy.loginfo(e)

        #displayMat("Final image", img)



    def __init__(self):
        rospy.init_node('image_overlayer_publisher')


        self.generated_image_topic = 'generated_image'

        self.filler_im_topic = "filler_image"
        self.main_im_topic = "main_image"

        rospy.loginfo("Hallucinated Robot Image Overlayer starting up")

        self.imgPub = rospy.Publisher(self.generated_image_topic, Image, queue_size=5)

        self.filler_im_sub = message_filters.Subscriber(self.filler_im_topic, Image)
        self.filler_im_cache = message_filters.Cache(self.filler_im_sub, 100)

        self.main_im_sub = rospy.Subscriber(self.main_im_topic, Image, self.mainImageCB)


        self.bridge = CvBridge()

        rospy.loginfo("Hallucinated Robot Image Overlayer ready")

        rospy.spin()


if __name__ == '__main__':
    try:
        ImageOverlayer()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
