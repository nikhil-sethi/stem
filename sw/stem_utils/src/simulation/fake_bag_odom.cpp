/*
This node helps in publishing the odometry with current timestamps because rviz wont display old odometry messages. annoying

*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
ros::Subscriber odom_sub;
ros::Publisher odom_pub;

void odomRepublisher(const nav_msgs::Odometry& msg){
    nav_msgs::Odometry msg_mod(msg);
    
    msg_mod.header.stamp = ros::Time::now();

    odom_pub.publish(msg_mod);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "fake_bag_odom_node");
    ros::NodeHandle nh;
    odom_sub = nh.subscribe("/mavros/local_position/odom", 10, odomRepublisher);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_topic", 1);

    ros::spin();
}