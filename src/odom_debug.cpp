#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_debug");
    ros::NodeHandle nh("~");
    
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    
    double x, y, z;
    nh.param("x", x, 1.0);
    nh.param("y", y, 0.0);
    nh.param("z", z, 0.0);
    
    ROS_INFO("Debug mode: sending fixed position [%.2f, %.2f, %.2f]", x, y, z);
    ROS_INFO("Check FC debug_test[13]=%.2f, debug_test[14]=%.2f", y, x);
    
    ros::Rate rate(10);
    while (ros::ok()) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose_pub.publish(pose);
        rate.sleep();
    }
    return 0;
}
