#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node('laser_frame_broadcaster')

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Set the parent and child frames for the TF transform
    parent_frame = "map"  # Change this to the desired parent frame
    child_frame = "laser"  # Change this to the name of your laser frame

    rate = rospy.Rate(10)  # Adjust the rate as needed for your application

    while not rospy.is_shutdown():
        # Create a TransformStamped message
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = parent_frame
        transform_stamped.child_frame_id = child_frame
        transform_stamped.transform.translation.x = 0.0  # Set the translation values
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = 0.0  # Set the rotation values
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0

        # Publish the transform
        tf_broadcaster.sendTransform(transform_stamped)

        rate.sleep()

if __name__ == '__main__':
    main()
