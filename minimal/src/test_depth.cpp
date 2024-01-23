#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "minimal/test_depth.h"
using namespace std;

TestDepth::TestDepth(ros::NodeHandle nh){
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/map_ros/depth", 50));
    pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "/map_ros/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<TestDepth::SyncPolicyImagePose>(
      TestDepth::SyncPolicyImagePose(30), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&TestDepth::depthPoseCallback, this, _1, _2));
  

    // pose_test_sub_ = nh.subscribe("/map_ros/pose", 50, &TestDepth::poseCallback, this);
    // depth_test_sub_ = nh.subscribe("/map_ros/depth", 50, &TestDepth::depthCallback, this);

    depth_pub_ = nh.advertise<sensor_msgs::Image>("depth_synced", 50);

    // proj_points_.resize(424 * 240 / (skip_pixel_ * skip_pixel_));
    // point_cloud_.points.resize(424 * 240 / (skip_pixel_ * skip_pixel_));
};

void TestDepth::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose){
    ROS_ERROR("in pose cb");
}

void TestDepth::depthCallback(const sensor_msgs::ImageConstPtr& img){
    depth
}

void TestDepth::depthPoseCallback(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose) {
    ROS_ERROR("sdfgsdfg");
    // camera_pos_(0) = pose->pose.position.x;
    // camera_pos_(1) = pose->pose.position.y;
    // camera_pos_(2) = pose->pose.position.z;
    // if (!map_->isInMap(camera_pos_))  // exceed mapped region
    //     return;

    // // Simulate swarm communication
    // map_->mm_->drone_pos_ = camera_pos_;

    // camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
    //     pose->pose.orientation.y, pose->pose.orientation.z);
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    // if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    //     (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
    // cv_ptr->image.copyTo(*depth_image_);

    // auto t1 = ros::Time::now();

    // // generate point cloud, update map
    // processDepthImage();
    // map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);

    // // ROS_WARN("Depth and inputcloud time: %lf", (ros::Time::now() - t1).toSec());

    // if (local_updated_) {
    //     // t1 = ros::Time::now();

    //     map_->clearAndInflateLocalMap();

    //     // ROS_WARN("Inflate time: %lf", (ros::Time::now() - t1).toSec());

    //     esdf_need_update_ = true;
    //     local_updated_ = false;
    // }

    // auto t2 = ros::Time::now();
    // fuse_time_ += (t2 - t1).toSec();
    // max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
    // fuse_num_ += 1;
    // if (show_occ_time_)
    //     ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
    //         max_fuse_time_);
}

// void MapROS::processDepthImage() {
//     proj_points_cnt = 0;

//     uint16_t* row_ptr;
//     int cols = depth_image_->cols;
//     int rows = depth_image_->rows;
//     double depth;
//     Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
//     Eigen::Vector3d pt_cur, pt_world;
//     const double inv_factor = 1.0 / k_depth_scaling_factor_;

//     for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
//         row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
//         for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
//         depth = (*row_ptr) * inv_factor;
//         row_ptr = row_ptr + skip_pixel_;

//         // // filter depth
//         // if (depth > 0.01)
//         //   depth += rand_noise_(eng_);

//         // TODO: simplify the logic here
//         // if (*row_ptr == 0) continue;

//         if (*row_ptr == 0 || depth > depth_filter_maxdist_)
//             depth = depth_filter_maxdist_;
//         else if (depth < depth_filter_mindist_)
//             continue;

//         pt_cur(0) = (u - cx_) * depth / fx_;
//         pt_cur(1) = (v - cy_) * depth / fy_;
//         pt_cur(2) = depth;
//         pt_world = camera_r * pt_cur + camera_pos_;
//         auto& pt = point_cloud_.points[proj_points_cnt++];
//         pt.x = pt_world[0];
//         pt.y = pt_world[1];
//         pt.z = pt_world[2];
//         }
//     }
//     ROS_ERROR("process depth");
//     publishDepth();
// }

// void MapROS::publishDepth() {
//     pcl::PointXYZ pt;
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     for (int i = 0; i < proj_points_cnt; ++i) {
//         cloud.push_back(point_cloud_.points[i]);
//     }
//     cloud.width = cloud.points.size();
//     cloud.height = 1;
//     cloud.is_dense = true;
//     cloud.header.frame_id = frame_id_;
//     sensor_msgs::PointCloud2 cloud_msg;
//     pcl::toROSMsg(cloud, cloud_msg);
//     depth_pub_.publish(cloud_msg);
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    TestDepth test(nh);
    ros::spin();
}