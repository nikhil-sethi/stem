
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

#include "minimal/test_depth.h"
using namespace std;

TestDepth::TestDepth(){
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/camera/depth/image_rect_raw", 50));
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/camera/pose", 25));
    
    sync_image_pose_.reset(new message_filters::Synchronizer<TestDepth::SyncPolicyImagePose>(
      TestDepth::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    // depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("depth_cloud", 10);
    sync_image_pose_->registerCallback(boost::bind(&TestDepth::depthPoseCallback, this, _1, _2));

    depth_sub = node_.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 50, &TestDepth::depthCallback, this);
    pose_sub = node_.subscribe<geometry_msgs::PoseStamped>("/camera/pose", 50, &TestDepth::poseCallback, this);

    pose_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/camera/pose", 1);
    // proj_points_.resize(424 * 240 / (skip_pixel_ * skip_pixel_));
    // point_cloud_.points.resize(424 * 240 / (skip_pixel_ * skip_pixel_));
};

void TestDepth::depthCallback(const sensor_msgs::ImageConstPtr& img) {
    ROS_ERROR("in depth");

    geometry_msgs::PoseStamped msg;
    msg.header = img->header;
    // pose_pub_.publish(msg);
}

void TestDepth::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
    ROS_ERROR("in pose");
}


void TestDepth::depthPoseCallback(
        const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose) {
    ROS_ERROR("in depth pose");
}

int main(int argc, char **argv){
    ros::init(argc, argv, "minimal_depth_tester");
    TestDepth test;
    ros::spin();
    return 0;
}