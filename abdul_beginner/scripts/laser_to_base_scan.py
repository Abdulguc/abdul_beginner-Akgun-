#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import tf2_ros
import numpy as np
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point

def get_rplidar_data(scan_msg):
    # Assuming you have a LaserScan message
    # Get the laser point from the scan data
    angle = scan_msg.angle_min
    range_value = scan_msg.ranges[0]  # For example, get the first range value
    laser_point = PointStamped()
    laser_point.header = scan_msg.header
    laser_point.point = Point(range_value * np.cos(angle), range_value * np.sin(angle), 0.0)

    # Create a TF2 Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Transform the laser point to the base_scan frame
    try:
        base_scan_frame = "base_scan"  # Change this to the actual frame name
        target_point = tf_buffer.transform(laser_point, base_scan_frame)
        rospy.loginfo("Transformed Point: %s", target_point)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Transformation failed!")

def main():
    rospy.init_node("laser_to_base_scan")

    # Subscribe to the LaserScan topic
    rospy.Subscriber("scan", LaserScan, get_rplidar_data)

    rospy.spin()

if __name__ == "__main__":
    main()
