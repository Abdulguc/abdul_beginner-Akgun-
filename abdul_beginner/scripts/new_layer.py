#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32


def set_costmap():
   pub = rospy.Publisher('/move_base/global_costmap/costmap', OccupancyGrid, queue_size = 10)
   rospy.init_node('set_costmap', anonymous = True)
   rate = rospy.Rate(10)
   
   while not rospy.is_shutdown():
     polygon = [Point32(0.25, 0.25, 0), Point32(0.75, 0.25, 0), Point32(0.25, 0.75, 0), Point32(0.75, 0.75, 0)]
     cost = 254
     
     msg = OccupancyGrid()
     msg.header.stamp = rospy.Time.now()
     msg.header.frame_id = 'map'
     msg.info.resolution = 0.05
     msg.info.width = 100
     msg.info.height = 100
     msg.info.origin.position.x = -msg.info.resolution * msg.info.width / 2.0
     msg.info.origin.position.y = -msg.info.resolution * msg.info.height / 2.0
     msg.info.origin.position.z = 0.0
        
     msg.data = np.array([-1] * (msg.info.width * msg.info.height)).astype(np.int8) 
        
     for i in range(len(polygon)):
        x = int(polygon[i].x / msg.info.resolution + msg.info.width / 2.0)
        y = int(polygon[i].y / msg.info.resolution + msg.info.height / 2.0)
        index = y * msg.info.width + x
        msg.data[index] = cost
            
     pub.publish(msg)
        
     rate.sleep()
        
if __name__ == '__main__':
   try:
     set_costmap() 
   except rospy.ROSInterruptException:
     pass
