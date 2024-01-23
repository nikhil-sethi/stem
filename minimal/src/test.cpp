// #include <opencv2/opencv.hpp>
#include <iostream>
#include "plan_env/map_ros.h"
#include <geometry_msgs/PoseStamped.h>
using namespace std;

class Test{
    public:
        Test(ros::NodeHandle nh){
            node_ = nh;
            sensor_pose_pub = node_.advertise<geometry_msgs::PoseStamped>("/drone0/camera", 10);
            timer = nh.createTimer(ros::Duration(0.1), &Test::pose_callback, this);
        }
        ros::Publisher sensor_pose_pub;
        ros::Timer timer;
        ros::NodeHandle node_;

    void pose_callback(const ros::TimerEvent&){
    
        geometry_msgs::PoseStamped sensor_pose = geometry_msgs::PoseStamped();
        sensor_pose.header.frame_id = "world";
        sensor_pose.pose.position.x = 0;
        sensor_pose.pose.position.x = 0;
        sensor_pose.pose.position.x = 0;
        sensor_pose.pose.orientation.x = 0;
        sensor_pose.pose.orientation.y = 0;
        sensor_pose.pose.orientation.z = 0;
        sensor_pose.pose.orientation.w = 1;

        sensor_pose_pub.publish(sensor_pose);

    }
};



int main(int argc, char **argv){
    
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    Test test(nh);
    
    fast_planner::MapROS* mr = new fast_planner::MapROS();
    mr->setNode(nh);
    mr->init();
    ROS_ERROR("asfdgadfg");
    ros::Duration(1.0).sleep();
    ros::spin();

}