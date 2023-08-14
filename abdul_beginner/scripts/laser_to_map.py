#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
import numpy as np

tf_buffer = None

def lidar_callback(lidar_data):
    try:
        global tf_buffer
        
        # Get the transform from laser_frame to map frame
        transform = tf_buffer.lookup_transform("map", lidar_data.header.frame_id, rospy.Time())

        # Process the LiDAR data
        for angle, distance in enumerate(lidar_data.ranges):
            x = distance * np.cos(lidar_data.angle_min + angle * lidar_data.angle_increment)
            y = distance * np.sin(lidar_data.angle_min + angle * lidar_data.angle_increment)

            # Transform the point from the laser frame to the map frame
            point = geometry_msgs.msg.PointStamped()
            point.header = lidar_data.header
            point.point.x = x
            point.point.y = y
            point_transformed = tf_buffer.transform(point, "map")

            # Now you can use point_transformed.point.x and point_transformed.point.y in the map frame
            x_map_frame = point_transformed.point.x
            y_map_frame = point_transformed.point.y

            # Your further processing here
            # ...

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Could not get transform between map and laser_frame.")

def main():
    rospy.init_node('laser_to_map_transform_node')
    
    global tf_buffer    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
