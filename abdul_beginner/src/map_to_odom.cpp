#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher transformed_map_pub; 

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Wait for the transform between /map and /odom to become available
    ros::Rate rate(10.0);  // 10 Hz
    /*while (ros::ok() && !tf_buffer.canTransform("odom", "map", ros::Time(0))) {
        ROS_INFO("Waiting for the transform between /map and /odom...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Transform between /map and /odom is available!");*/

    // Transform the entire map from /map to /odom frame
    nav_msgs::OccupancyGrid transformed_map = *map;
    for (size_t i = 0; i < map->data.size(); ++i) {
        // Assuming the map resolution is 1 cell per meter
        geometry_msgs::PointStamped map_point, odom_point;
        map_point.header.frame_id = "map";
        map_point.point.x = (i % map->info.width) * map->info.resolution;
        map_point.point.y = (i / map->info.width) * map->info.resolution;
        map_point.point.z = 0.0;

        try {
            tf_buffer.transform(map_point, odom_point, "odom");
            int transformed_index = (int(odom_point.point.y / map->info.resolution) * map->info.width) +
                                    int(odom_point.point.x / map->info.resolution);
            if (transformed_index >= 0 && transformed_index < map->data.size()) {
                transformed_map.data[transformed_index] = map->data[i];
            }
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Failed to transform map cell %zu: %s", i, ex.what());
        }
    }

    // Publish the transformed map
    transformed_map_pub.publish(transformed_map);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_to_odom_transformer");

    ros::NodeHandle nh;
    transformed_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/transformed_map", 1000);
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);
    
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = -10.0;
    transformStamped.transform.translation.y = -10.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    
    tf_broadcaster.sendTransform(transformStamped);

    ros::spin();

    return 0;
}
