#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import laser_geometry.laser_geometry as lg

class LaserScanToPointCloud:
    def __init__(self):
        self.laser_projector = lg.LaserProjection()
        self.tf_listener = tf.TransformListener()
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.pc_pub = rospy.Publisher("/converted_pointcloud", PointCloud2, queue_size=1)

    def scan_callback(self, scan_msg):
        rospy.loginfo("Scan received, converting to PointCloud2...")
        try:
            cloud = self.laser_projector.projectLaser(scan_msg)
            self.pc_pub.publish(cloud)
        except Exception as e:
            rospy.logwarn("Projection failed: {}".format(e))

if __name__ == '__main__':
    rospy.init_node("laser_to_pointcloud_node")
    LaserScanToPointCloud()
    rospy.spin()
