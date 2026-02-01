#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class OdomToAcfly {
public:
    OdomToAcfly() : nh_("~"), tf_buffer_(), tf_listener_(tf_buffer_) {
        nh_.param<std::string>("odom_topic", odom_topic_, "/odin1/odometry_highfreq");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("fcu_frame", fcu_frame_, "fcu_link");
        
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &OdomToAcfly::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        try {
            auto tf = tf_buffer_.lookupTransform(odom_frame_, fcu_frame_, ros::Time(0), ros::Duration(0.1));
            
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = msg->header.stamp;
            pose.header.frame_id = odom_frame_;
            pose.pose.position.x = tf.transform.translation.x;
            pose.pose.position.y = tf.transform.translation.y;
            pose.pose.position.z = tf.transform.translation.z;
            pose.pose.orientation = tf.transform.rotation;
            
            pose_pub_.publish(pose);
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "%s", ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Subscriber odom_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string odom_topic_, odom_frame_, fcu_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_acfly");
    OdomToAcfly node;
    ros::spin();
    return 0;
}