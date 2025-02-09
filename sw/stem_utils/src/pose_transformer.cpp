#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PoseTransformer {
public:
    PoseTransformer() : nh_("~") {
        // Initialize parameters
        nh_.param<std::string>("input_pose_topic", input_pose_topic_, "/mavros/local_position/odom");
        nh_.param<std::string>("output_pose_topic", output_pose_topic_, "/camera/pose");
        nh_.param<std::string>("parent_frame", parent_frame_, "odom");
        nh_.param<std::string>("child_frame", child_frame_, "camera_link");


        ROS_INFO_STREAM(""<<input_pose_topic_);
        // Subscribe to input pose topic
        pose_sub_ = nh_.subscribe(input_pose_topic_, 1, &PoseTransformer::poseCallback, this);

        // Advertise output pose topic
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_pose_topic_, 1);

        // Initialize transform listener
        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    }

    ~PoseTransformer() {
        delete tf_listener_;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
        // Transform the pose
        geometry_msgs::TransformStamped transformStamped;
        ROS_INFO_STREAM("Sdgsd");
        // try {
        //     transformStamped = tf_buffer_.lookupTransform(parent_frame_, child_frame_, ros::Time(0));
        // } catch (tf2::TransformException &ex) {
        //     ROS_WARN("Transform lookup failed: %s", ex.what());
        //     return;
        // }
        
        geometry_msgs::PoseStamped transformed_pose;
        transformed_pose.header = pose_msg->header;
        tf2::doTransform(*pose_msg, transformed_pose, transformStamped);

        // Publish the transformed pose
        pose_pub_.publish(transformed_pose);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher pose_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_;
    std::string input_pose_topic_;
    std::string output_pose_topic_;
    std::string parent_frame_;
    std::string child_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_transformer");
    PoseTransformer transformer;
    ros::spin();
    return 0;
}
