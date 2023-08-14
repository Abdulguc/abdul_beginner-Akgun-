#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import LaserScan

def laser_callback(laser_data):
    global tf_buffer
    
    # Assume robot's initial pose in map frame is (0, 0, 0)
    x_map = 0.0
    y_map = 0.0
    
    # Loop through laser scan data and transform each point to map frame
    for angle, distance in enumerate(laser_data.ranges):
        if distance < laser_data.range_max:
            # Calculate point's x and y in base_link frame
            x_base_link = distance * np.cos(laser_data.angle_min + angle * laser_data.angle_increment)
            y_base_link = distance * np.sin(laser_data.angle_min + angle * laser_data.angle_increment)

            # Transform point to map frame
            transform = tf_buffer.lookup_transform("map", laser_data.header.frame_id, rospy.Time())
            x_map_frame = transform.transform.translation.x + x_base_link
            y_map_frame = transform.transform.translation.y + y_base_link

            # Now you have the point's coordinates in the map frame
            # Use x_map_frame and y_map_frame for further processing

def main():
    rospy.init_node('laser_to_map_transform_node')
    
    global tf_buffer
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
