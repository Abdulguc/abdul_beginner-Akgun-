#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

def processFeedback(feedback):
	if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
		print("Button clicked!")
		
if __name__ == "__main__":
	rospy.init_node('draw_square')

	server = InteractiveMarkerServer("simple_marker")

#	marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

	int_marker = InteractiveMarker()
	int_marker.header.frame_id = "base_link"
	int_marker.name = "my_marker"
	int_marker.pose.position.x = 0.0
	int_marker.pose.position.y = 0.0
	int_marker.pose.position.z = 0.5

	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.type = marker.CUBE
	marker.action = marker.ADD

	marker.scale.x = 0.5
	marker.scale.y = 0.5
	marker.scale.z = 0.05
	marker.color.r = 0.5
	marker.color.g = 0.5
	marker.color.b = 0.5
	marker.color.a = 1.0

	marker_control = InteractiveMarkerControl()
	marker_control.always_visible = True
	marker_control.markers.append(marker)

	int_marker.controls.append(marker_control)

	button_control = InteractiveMarkerControl()
	button_control.name = "button_control"
	button_control.interaction_mode = InteractiveMarkerControl.BUTTON	

	int_marker.controls.append(button_control)
	
	feedback_msg = InteractiveMarkerFeedback()
	feedback_msg.event_type = InteractiveMarkerFeedback.BUTTON_CLICK
	
	processFeedback(feedback_msg)

	server.insert(int_marker, processFeedback)

	server.applyChanges()
	
	rospy.spin()

"""	while not rospy.is_shutdown():
		marker_pub.publish(marker)
		rospy.sleep(1)"""
"""	p1 = Point()
	p1.x = 0.0
	p1.y = 0.0
	p1.z = 0.0

	p2 = Point()
	p2.x = 1.0
	p2.y = 0.0
	p2.z = 0.0

	p3 = Point()
	p3.x = 1.0
	p3.y = 1.0
	p3.z = 0.0

	p4 = Point()
	p4.x = 0.0
	p4.y = 1.0
	p4.z = 0.0

	marker.points.append(p1)
	marker.points.append(p2)
	marker.points.append(p3)
	marker.points.append(p4)
	marker.points.append(p1)"""
