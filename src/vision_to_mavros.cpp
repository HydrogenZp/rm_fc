#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

Eigen::Vector3d p_mav;
Eigen::Quaterniond q_mav;
/**
 * @brief Callback function for VINS odometry messages
 *
 * @param msg
 */
void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id != "world")
        return;

    double x_north = msg->pose.pose.position.x; // NWU.x = North
    double y_west = msg->pose.pose.position.y;  // NWU.y = West
    double z_up = msg->pose.pose.position.z;    // NWU.z = Up

    p_mav = Eigen::Vector3d(-y_west, x_north, z_up);

    Eigen::Quaterniond q_nwu(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    Eigen::Quaterniond q_nwu_to_enu;
    q_nwu_to_enu = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());

    q_mav = q_nwu_to_enu * q_nwu;
}

int main(int argc, char **argv)
{
    /**
     * @brief ROS Node Initialization
     *
     */
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh("~");

    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("odom", 100, vins_callback);
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);

    ros::Rate rate(30);

    /**
     * @brief Construct a new while object
     *
     */
    while (ros::ok())
    {
        geometry_msgs::PoseStamped vision;
        vision.header.frame_id = "map";
        vision.header.stamp = ros::Time::now();

        vision.pose.position.x = -p_mav[0];
        vision.pose.position.y = -p_mav[1];
        vision.pose.position.z = p_mav[2];

        vision.pose.orientation.x = q_mav.x();
        vision.pose.orientation.y = q_mav.y();
        vision.pose.orientation.z = q_mav.z();
        vision.pose.orientation.w = q_mav.w();

        vision_pub.publish(vision);

        ROS_INFO_THROTTLE(1.0, "\nposition:\n   x: %.6f\n   y: %.6f\n   z: %.6f\n"
                               "orientation:\n   x: %.6f\n   y: %.6f\n   z: %.6f\n   w: %.6f",
                          p_mav[0], p_mav[1], p_mav[2],
                          q_mav.x(), q_mav.y(), q_mav.z(), q_mav.w());

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}