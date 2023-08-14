#!/usr/bin/env python3

import rospy
import tf
import math
from geometry_msgs.msg import Quaternion, TransformStamped, Twist

def main():
    rospy.init_node('sol_teker')

    odom_broadcaster = tf.TransformBroadcaster()

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0
    vy = 0.0
    vth = 0.000001

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
    
        current_time = rospy.Time.now()

        # Compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        # Create a quaternion from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # Publish transform over tf
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "base_link"
        odom_trans.child_frame_id = "wheel_left_link"

        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        odom_broadcaster.sendTransformMessage(odom_trans)

        last_time = current_time
        rate.sleep()

if __name__ == '__main__':
    main()
