#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class ScanToCloud:
    def __init__(self):
        rospy.init_node('scan_to_cloud_node')
        self.laser_projector = LaserProjection()
        self.pub = rospy.Publisher('/converted_cloud', PointCloud2, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        rospy.loginfo("Scan to PointCloud2 node started.")
        rospy.spin()

    def callback(self, scan_msg):
        cloud = self.laser_projector.projectLaser(scan_msg)
        cloud.header = scan_msg.header
        self.pub.publish(cloud)

if __name__ == '__main__':
    ScanToCloud()
