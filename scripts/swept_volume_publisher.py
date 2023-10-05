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


def displayMat(name, image):
    minv, maxv, minloc, maxloc = cv2.minMaxLoc(image)

    copy = image.copy()
    copy[np.where(copy != copy)] = 0

    viz = copy * 1.0 / maxv

    cv2.imshow(name, viz)
    cv2.waitKey(1)

class SweptVolumePublisher():


    def imageCB(self, image_msg):
        rospy.logdebug("Hallucinated image callback")

        target_time = image_msg.header.stamp
        camera_infos = self.camera_info_cache.getInterval(target_time, target_time)

        rospy.logdebug("Target time:" + str(target_time))

        if len(camera_infos) > 0:
            self.camera_model.fromCameraInfo(camera_infos[0])

            rospy.logdebug("Info available")

            try:
                image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

                real_image = np.zeros_like(image, dtype=np.uint8)

                if image.dtype == np.uint16:
                    real_image[np.where(image > 0)] = 255
                    scale = .001
                elif image.dtype == np.float32:
                    real_image[np.where(image == image)] = 255
                    scale = 1
                else:
                    rospy.logerr("Unexpected data type for image! " + str(image.dtype))
                    return

                contour_ret, contours, hierarchy = cv2.findContours(real_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #probably use RETR_LIST since we don't need hierarchy. APPROX_SIMPLE: looks good, except for curves
                #Note: Contour points use (row, col) notation

                rospy.logdebug("Got contours")

                marker = Marker()
                marker.type = Marker.TRIANGLE_LIST
                marker.action = Marker.ADD
                marker.header = image_msg.header

                marker.scale = Vector3(x=scale,y=scale,z=scale)

                marker.ns="swept volume"

                marker.color = ColorRGBA(a=.25, b = 1)
                #Note: The following don't need to be changed
                # marker.pose
                # marker.id=0

                points = marker.points

                origin = [0,0,0]

                origin = Point(*origin)

                if self.debug:
                    contour_values = np.zeros_like(image)

                def pointToPointMsg(point):
                    pnt = Point(*point)
                    return pnt

                def rect_contains(rect, point):
                    if point[0] < rect[0]:
                        return False
                    elif point[1] < rect[1]:
                        return False
                    elif point[0] > rect[2]:
                        return False
                    elif point[1] > rect[3]:
                        return False
                    return True

                def pixelToPoint(uv):
                    if isinstance(uv, np.ndarray):
                        uv = uv.flatten()    #Note: use ravel() for view

                    coord = (uv[1], uv[0])
                    depth = image[coord]

                    if depth == depth:

                        #Note: the ray is a unit vector, unlike the cpp implementation, which sets z=1.0. Sources:
                        # https://github.com/ros-perception/vision_opencv/blob/kinetic/image_geometry/src/image_geometry/cameramodels.py#L142
                        # https://github.com/ros-perception/vision_opencv/blob/kinetic/image_geometry/src/pinhole_camera_model.cpp#L289
                        ray = self.camera_model.projectPixelTo3dRay(uv=uv)
                        ray = np.asarray(ray)
                        point = ray * depth / ray[2]

                        point = point.tolist()

                        if self.debug:
                            # This was used to verify that the contour is located on the 'true' side of the boundary
                            contour_values[coord] = depth

                        point = Point(*point)

                        return point
                    else:
                        return None


                for contour in contours:

                    num_pnts = len(contour)
                    if num_pnts > 1:
                        first_pnt = pixelToPoint(contour[0])
                        prev_pnt = first_pnt

                        for i in range(1,num_pnts):
                            pass
                            new_pnt = pixelToPoint(contour[i])

                            points += [prev_pnt, new_pnt, origin]
                            prev_pnt = new_pnt

                        points += [prev_pnt, first_pnt, origin]

                rospy.logdebug("Added contour points")

                self.marker_pub.publish(marker)
                rospy.logdebug("Publishing marker")


                if self.debug:
                    displayMat("image", image)
                    displayMat("mask", real_image)

                    contour_im = np.zeros_like(image, dtype=np.uint8)
                    cv2.drawContours(contour_im, contours, -1, (255), 1)
                    displayMat("contours", contour_im)
                    displayMat("contour values", contour_values)


            except CvBridgeError as e:
                rospy.loginfo(e)

        #displayMat("Final image", img)



    def __init__(self):
        rospy.init_node('swept_volume_publisher')

        self.debug = False

        self.visualization_topic = 'swept_volume'

        self.image_topic = 'image_in'

        rospy.loginfo("Swept volume publisher starting up")

        self.camera_model = PinholeCameraModel()
        self.marker_pub = rospy.Publisher(self.visualization_topic, Marker, queue_size=5)

        self.im_sub = rospy.Subscriber(self.image_topic, Image, self.imageCB)

        self.camera_info_topic = "camera_info"
        self.camera_info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        self.camera_info_cache = message_filters.Cache(self.camera_info_sub, 100)

        self.bridge = CvBridge()

        rospy.loginfo("Hallucinated Robot Swept Volume Publisher ready")

        rospy.spin()


if __name__ == '__main__':
    try:
        SweptVolumePublisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
