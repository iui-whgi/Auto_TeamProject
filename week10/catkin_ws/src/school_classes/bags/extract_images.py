#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False
        rospy.Subscriber("/camera/image", Image, self.callback)

    def callback(self, msg):
        if not self.image_received:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv2.imwrite("saved_image.jpg", cv_image)
                rospy.loginfo("Image saved as saved_image.jpg")
                self.image_received = True
            except Exception as e:
                rospy.logwarn("Failed to convert image: {}".format(e))

if __name__ == "__main__":
    rospy.init_node("image_saver_node")
    ImageSaver()
    rospy.spin()
