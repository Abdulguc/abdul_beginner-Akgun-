#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import TransformStamped

def map_callback(map):
	tf_buffer = tf2_ros.Buffer()
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	# Wait for the transform between /map and /odom to become available
	rate = rospy.Rate(10.0)  # 10 Hz
	while not rospy.is_shutdown() and not tf_buffer.can_transform("odom", "map", rospy.Time(0)):
		rospy.loginfo("Waiting for the transform between /map and /odom...")
		rate.sleep()

	rospy.loginfo("Transform between /map and /odom is available!")

	# Transform the entire map from /map to /odom frame
	transformed_map = map
	for i in range(len(map.data)):
		# Assuming the map resolution is 1 cell per meter
		map_point = PointStamped()
		map_point.header.frame_id = "map"
		map_point.point.x = (i % map.info.width) * map.info.resolution
		map_point.point.y = (i / map.info.width) * map.info.resolution
		map_point.point.z = 0.0

		try:
			odom_point = tf_buffer.transform(map_point, "odom", rospy.Duration(1.0))
			transformed_index = int(odom_point.point.y / map.info.resolution) * map.info.width + int(odom_point.point.x / map.info.resolution)
			if 0 <= transformed_index < len(map.data):
				transformed_map_list = list(transformed_map.data)			
				transformed_map_list[transformed_index] = map.data[i]
				transformed_map.data = tuple(transformed_map_list)
		except tf2_ros.TransformException as ex:
			rospy.logwarn("Failed to transform map cell %s: %s", i, ex)

	# Publish the transformed map
	transformed_map_pub.publish(transformed_map)

"""def publish_odom_frame():
	rospy.init_node("odom_frame_publisher")

	tf_broadcaster = tf2_ros.TransformBroadcaster()
	transform_stamped = TransformStamped()

	transform_stamped.header.stamp = rospy.Time.now()
	transform_stamped.header.frame_id = "map"  # Parent frame (change this to match your setup)
	transform_stamped.child_frame_id = "odom"
	transform_stamped.transform.translation.x = 1.0  # Set the desired translation values
	transform_stamped.transform.translation.y = 2.0
	transform_stamped.transform.translation.z = 0.0
	transform_stamped.transform.rotation.x = 0.0  # Set the desired rotation values
	transform_stamped.transform.rotation.y = 0.0
	transform_stamped.transform.rotation.z = 0.0
	transform_stamped.transform.rotation.w = 1.0

	rate = rospy.Rate(10)  # Broadcast at 10 Hz
	while not rospy.is_shutdown():
		# Update the timestamp and broadcast the transformation
		transform_stamped.header.stamp = rospy.Time.now()
		tf_broadcaster.sendTransform(transform_stamped)
		rate.sleep()"""
if __name__ == "__main__":
	rospy.init_node("map_to_odom_transformer")
	try:
		transformed_map_pub = rospy.Publisher("/transformed_map", OccupancyGrid, queue_size=1)
		map_sub = rospy.Subscriber("/map", OccupancyGrid, map_callback)
		#publish_odom_frame()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
