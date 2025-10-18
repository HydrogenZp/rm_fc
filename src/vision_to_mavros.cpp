#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher vision_pub;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

/**
 * @brief Callback function for VINS odometry messages
 *
 * @param msg
 */
void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id != "world")
        return;

    try
    {
        // 查找从body到fcu_link的变换
        geometry_msgs::TransformStamped transform_stamped =
            tf_buffer->lookupTransform("fcu_link", "body", ros::Time(0));

        // 创建body系的位姿
        geometry_msgs::PoseStamped body_pose;
        body_pose.header = msg->header;
        body_pose.header.frame_id = "body";
        body_pose.pose = msg->pose.pose;

        // 通过TF变换到fcu_link系
        geometry_msgs::PoseStamped fcu_pose;
        tf2::doTransform(body_pose, fcu_pose, transform_stamped);

        // 发布变换后的位姿
        geometry_msgs::PoseStamped vision;
        vision.header.frame_id = "map";
        vision.header.stamp = ros::Time::now();
        vision.pose = fcu_pose.pose;

        vision_pub.publish(vision);

        ROS_INFO_THROTTLE(1.0, "\nfcu_link position:\n   x: %.6f\n   y: %.6f\n   z: %.6f\n"
                               "orientation:\n   x: %.6f\n   y: %.6f\n   z: %.6f\n   w: %.6f",
                          vision.pose.position.x, vision.pose.position.y, vision.pose.position.z,
                          vision.pose.orientation.x, vision.pose.orientation.y,
                          vision.pose.orientation.z, vision.pose.orientation.w);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(1.0, "Transform lookup failed: %s", ex.what());
    }
}

int main(int argc, char **argv)
{
    /**
     * @brief ROS Node Initialization
     *
     */
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh("~");

    // 初始化TF
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // 发布静态TF: body到fcu_link的变换
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transform;

    // 从参数服务器读取变换参数（body系相机到fcu的偏移）
    double tx, ty, tz, qx, qy, qz, qw;
    nh.param("transform/x", tx, 0.0);
    nh.param("transform/y", ty, 0.0);
    nh.param("transform/z", tz, 0.0);
    nh.param("transform/qx", qx, 0.0);
    nh.param("transform/qy", qy, 0.0);
    nh.param("transform/qz", qz, 0.0);
    nh.param("transform/qw", qw, 1.0);

    static_transform.header.stamp = ros::Time::now();
    static_transform.header.frame_id = "body";
    static_transform.child_frame_id = "fcu_link";
    static_transform.transform.translation.x = tx;
    static_transform.transform.translation.y = ty;
    static_transform.transform.translation.z = tz;
    static_transform.transform.rotation.x = qx;
    static_transform.transform.rotation.y = qy;
    static_transform.transform.rotation.z = qz;
    static_transform.transform.rotation.w = qw;

    static_broadcaster.sendTransform(static_transform);

    ROS_INFO("Published static TF from body to fcu_link:");
    ROS_INFO("  Translation: [%.3f, %.3f, %.3f]", tx, ty, tz);
    ROS_INFO("  Rotation: [%.3f, %.3f, %.3f, %.3f]", qx, qy, qz, qw);

    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("odom", 100, vins_callback);
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);

    ros::spin();

    return 0;
}