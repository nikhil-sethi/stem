#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Tf2Topic {
public:
    Tf2Topic() : nh_("~") {
        // Initialize parameters
        nh_.param<std::string>("output_pose_topic", output_pose_topic_, "/camera/pose");
        nh_.param<std::string>("parent_frame", parent_frame_, "odom");
        nh_.param<std::string>("child_frame", child_frame_, "camera_link");
        
        // Advertise output pose topic
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_pose_topic_, 1);

        timer = nh_.createTimer(ros::Duration(0.05), &Tf2Topic::publishTransform, this);

        // Initialize transform listener
        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    }

    ~Tf2Topic() {
        delete tf_listener_;
    }

    void publishTransform(const ros::TimerEvent& event) {
        // Transform the pose
        geometry_msgs::TransformStamped transformStamped;
        ROS_INFO_STREAM("Sdgsd");
        try {
            transformStamped = tf_buffer_.lookupTransform(parent_frame_, child_frame_, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform lookup failed: %s", ex.what());
            return;
        }
        
         geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = transformStamped.header.stamp;
        pose_msg.header.frame_id = parent_frame_;
        pose_msg.pose.position.x = transformStamped.transform.translation.x;
        pose_msg.pose.position.y = transformStamped.transform.translation.y;
        pose_msg.pose.position.z = transformStamped.transform.translation.z;
        pose_msg.pose.orientation = transformStamped.transform.rotation;
        ROS_INFO("Sdg");
        pose_pub_.publish(pose_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Timer timer;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_;
    std::string output_pose_topic_;
    std::string parent_frame_;
    std::string child_frame_;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "tf2topic");
    Tf2Topic tf2topic;
    ros::spin();
    return 0;

    return 0;
}
