#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher vision_pub;

/**
 * @brief Callback function for VINS odometry messages
 */
void vins_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::PoseStamped vision;

  vision.header = msg->header;
  vision.pose = msg->pose.pose;
  vision_pub.publish(vision);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vins_to_mavros_converter");
  ros::NodeHandle nh("~");
  ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/vins_node/odometry", 100, vins_callback);

  vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

  ROS_INFO("VINS to MAVROS Direct Relay Node Started.");

  ros::spin();

  return 0;
}