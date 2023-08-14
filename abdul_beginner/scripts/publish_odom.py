#!/usr/bin/env python3

import rospy
import math
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == "__main__":
    rospy.init_node("static_tf_publisher")

    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    
#    angle = -(math.pi / 2.0)
#    q = tf.transformations.quaternion_from_euler(angle, 0.0, 0.0)

    transforms = []

    # Create and add the transform for odom frame
    odom_transform = TransformStamped()
    odom_transform.header.frame_id = "map"  # Parent frame
    odom_transform.child_frame_id = "odom"
    odom_transform.transform.translation.x = 0.0
    odom_transform.transform.translation.y = 0.0
    odom_transform.transform.translation.z = 0.0
    odom_transform.transform.rotation.x = 0.0
    odom_transform.transform.rotation.y = 0.0
    odom_transform.transform.rotation.z = 0.0
    odom_transform.transform.rotation.w = 1.0
    transforms.append(odom_transform)

    # Create and add the transform for wheel_right_link
#    wheel_right_transform = TransformStamped()
#    wheel_right_transform.header.frame_id = "base_link"  # Parent frame
#    wheel_right_transform.child_frame_id = "wheel_right_link"
#    wheel_right_transform.transform.translation.x = 0.0
#    wheel_right_transform.transform.translation.y = -0.08
#    wheel_right_transform.transform.translation.z = 0.023
#    wheel_right_transform.transform.rotation.x = q[0]
#    wheel_right_transform.transform.rotation.y = q[1]
#    wheel_right_transform.transform.rotation.z = q[2]
#    wheel_right_transform.transform.rotation.w = q[3]
#    transforms.append(wheel_right_transform)

    # Create and add the transform for wheel_left_link
#    wheel_left_transform = TransformStamped()
#    wheel_left_transform.header.frame_id = "base_link"  # Parent frame
#    wheel_left_transform.child_frame_id = "wheel_left_link"
#    wheel_left_transform.transform.translation.x = 0.0
#    wheel_left_transform.transform.translation.y = 0.08
#    wheel_left_transform.transform.translation.z = 0.023
#    wheel_left_transform.transform.rotation.x = q[0]
#    wheel_left_transform.transform.rotation.y = q[1]
#    wheel_left_transform.transform.rotation.z = q[2]
#    wheel_left_transform.transform.rotation.w = q[3]
#    transforms.append(wheel_left_transform)

    static_broadcaster.sendTransform(transforms)
    
    rospy.spin()
