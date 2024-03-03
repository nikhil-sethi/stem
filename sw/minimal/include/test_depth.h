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

using namespace std;

class TestDepth{
    public:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyImagePose;
        typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
        SynchronizerImagePose sync_image_pose_;
        ros::Publisher depth_pub_;
        shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
        vector<Eigen::Vector3d> proj_points_;
        ros::NodeHandle node_;

    public:
        TestDepth();

        // void MapROS::depthPoseCallback(
        //         const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);

        // void MapROS::processDepthImage();
        
        // void MapROS::publishDepth();

};


int main(int argc, char **argv){
    uint8_t a = 5;
    int b = 4;
    std::cout<<a+b<<std::endl;    
}