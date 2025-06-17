#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class WallAligner:
    def __init__(self):
        rospy.init_node('wall_aligner')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.twist = Twist()
        self.wall_parallel = False
        self.rate = rospy.Rate(10)
        self.start_align_check_time = rospy.Time.now() + rospy.Duration(7.0)  # 2초 회전 + 5초 직진

    def rotate_left(self, angular_speed, duration):
        self.twist.linear.x = 0.0
        self.twist.angular.z = angular_speed
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.pub.publish(self.twist)
            self.rate.sleep()
        self.stop()

    def move_forward(self, linear_speed, duration):
        self.twist.linear.x = linear_speed
        self.twist.angular.z = 0.0
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.pub.publish(self.twist)
            self.rate.sleep()
        self.stop()

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)

    def scan_callback(self, msg):
        if rospy.Time.now() < self.start_align_check_time or self.wall_parallel:
            return

        num_ranges = len(msg.ranges)
        center_index = int((math.radians(270) - msg.angle_min) / msg.angle_increment)
        half_window = int(math.radians(30) / msg.angle_increment)

        start_index = max(0, center_index - half_window)
        end_index = min(num_ranges, center_index + half_window)

        ranges = []

        for i in range(start_index, end_index):
            dist = msg.ranges[i]
            if not math.isinf(dist) and not math.isnan(dist):
                ranges.append(dist)

        if len(ranges) >= 2:
            diff = max(ranges) - min(ranges)
            if diff < 0.05:
                self.wall_parallel = True
                self.stop()
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.1
                self.pub.publish(self.twist)

    def run(self):
        self.rotate_left(0.36, 2.0)
        self.move_forward(0.8, 5.0)

        rospy.loginfo("Start wall alignment...")
        while not rospy.is_shutdown() and not self.wall_parallel:
            self.rate.sleep()
        rospy.loginfo("Wall aligned.")

        self.move_forward(0.8, 2.4)

        # 추가: 마지막 회전 + 직진
        self.rotate_left(0.39, 2.0)
        self.move_forward(0.8, 2.0)


if __name__ == '__main__':
    aligner = WallAligner()
    aligner.run()
