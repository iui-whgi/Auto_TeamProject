# #!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2

# class ImageSaver:
#     def __init__(self):
#         rospy.init_node('image_saver_node')
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
#         self.current_image = None
#         rospy.loginfo("Subscribed to /camera/image")

#     def image_callback(self, msg):
#         try:
#             self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             rospy.loginfo_once("Image received.")  # 첫 이미지 수신 시 1회 출력
#         except Exception as e:
#             rospy.logerr(f"cv_bridge error: {e}")

#     def wait_and_save(self):
#         rospy.loginfo("Press 'q' in terminal and press [Enter] to save image.")
#         while not rospy.is_shutdown():
#             user_input = input()
#             if user_input.strip().lower() == 'q':
#                 if self.current_image is not None:
#                     filename = "captured_image.png"
#                     cv2.imwrite(filename, self.current_image)
#                     rospy.loginfo(f"Image saved as {filename}")
#                 else:
#                     rospy.logwarn("No image received yet.")
#                 break

# if __name__ == '__main__':
#     saver = ImageSaver()
#     saver.wait_and_save()

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver_node')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.current_image = None
        rospy.loginfo("Subscribed to /camera/image")

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rospy.loginfo_once("Image received.")  # Log only once
        except Exception as e:
            rospy.logerr(f"cv_bridge error: {e}")

    def wait_and_save(self):
        rospy.loginfo("Type 'q' and press Enter anytime to save the current image. Press Ctrl+C to exit.")

        while not rospy.is_shutdown():
            user_input = input()
            if user_input.strip().lower() == 'q':
                if self.current_image is not None:
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"captured_{timestamp}.png"
                    cv2.imwrite(filename, self.current_image)
                    rospy.loginfo(f"Image saved as {filename}")
                else:
                    rospy.logwarn("No image received yet.")

if __name__ == '__main__':
    saver = ImageSaver()
    saver.wait_and_save()
