#include <iostream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <memory>

using namespace std;

class TestDepth{
    public:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyImagePose;
        typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
        SynchronizerImagePose sync_image_pose_;
        ros::Publisher pose_pub_;
        shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
        shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
        ros::Subscriber depth_sub, pose_sub;
        ros::NodeHandle node_;

    public:
        TestDepth();

        void depthPoseCallback(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
        void depthCallback(const sensor_msgs::ImageConstPtr& img);
        void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
};