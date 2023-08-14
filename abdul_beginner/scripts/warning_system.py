#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan

class WarningSystemNode:
    def __init__(self):
        rospy.init_node('warning_system_node', anonymous=True)

        # Set the private area boundaries (defined as a polygon here).
        self.private_area_x = [1.0, 2.0, 2.0, 1.0]
        self.private_area_y = [1.0, 1.0, 2.0, 2.0]

        # Subscribe to the LiDAR topic (adjust topic name if necessary).
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

    def is_inside_private_area(self, x, y):
        # Check if a given point (x, y) is inside the private area polygon.
        n = len(self.private_area_x)
        j = n - 1
        odd_nodes = False

        for i in range(n):
            if (self.private_area_y[i] < y and self.private_area_y[j] >= y) or \
               (self.private_area_y[j] < y and self.private_area_y[i] >= y):
                if self.private_area_x[i] + (y - self.private_area_y[i]) / \
                   (self.private_area_y[j] - self.private_area_y[i]) * \
                   (self.private_area_x[j] - self.private_area_x[i]) < x:
                    odd_nodes = not odd_nodes
            j = i

        return odd_nodes

    def lidar_callback(self, data):
        # Process the LiDAR data to detect objects within the private area.
        for angle, distance in enumerate(data.ranges):
            x = distance * math.cos(data.angle_min + angle * data.angle_increment)
            y = distance * math.sin(data.angle_min + angle * data.angle_increment)

            if self.is_inside_private_area(x, y):
                rospy.logwarn("Warning: Object detected in private area!")

def main():
    try:
        warning_system = WarningSystemNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
