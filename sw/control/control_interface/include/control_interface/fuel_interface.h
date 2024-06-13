#include "drone_toolbox_ext_control_template/controller.h"
#include <ros/ros.h>
#include "quadrotor_msgs/PositionCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include  <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class FUELInterface: public Controller{
    public:
        FUELInterface(ros::NodeHandle& nh);
        void controllerExecution();
        void cmdCallback(const quadrotor_msgs::PositionCommand msg);
        bool enableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        void sensorPoseTimer(const ros::TimerEvent& event);


    private:
        ros::Subscriber cmd_sub_;
        ros::Publisher pose_pub_,fuel_trigger_pub_, sensor_pose_pub;
        geometry_msgs::PoseStamped pose;
        ros::Timer sensor_pose_timer;
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener tf_listener;

};


FUELInterface::FUELInterface(ros::NodeHandle& nh):Controller(nh), tf_listener(buffer){
    cmd_sub_ = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 1, &FUELInterface::cmdCallback, this);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    fuel_trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    sensor_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera/pose", 10);

    // tf2_ros::TransformListener tf_listener(buffer);
    sensor_pose_timer = nh.createTimer(ros::Duration(0.05), &FUELInterface::sensorPoseTimer, this);
}


void FUELInterface::cmdCallback(const quadrotor_msgs::PositionCommand msg){
    // pose.header = msg.header;
    // pose.header.frame_id = "world";
    // pose.pose.position.x = msg.position.x;
    // pose.pose.position.y = msg.position.y;
    // pose.pose.position.z = msg.position.z;

    // tf2::Quaternion quat;

    // quat.setRPY(0,0,msg.yaw);
    // quat = quat.normalize();
    // pose.pose.orientation.w = quat.getW();
    // pose.pose.orientation.x = quat.getX();
    // pose.pose.orientation.y = quat.getY();
    // pose.pose.orientation.z = quat.getZ();
    pos_target_msg_.header = msg.header;
    pos_target_msg_.type_mask = 0;
    if (msg.trajectory_flag == 0)
        pos_target_msg_.type_mask = 2552;    
    pos_target_msg_.position = msg.position;
    pos_target_msg_.velocity = msg.velocity;
    pos_target_msg_.acceleration_or_force = msg.acceleration;
    pos_target_msg_.yaw = msg.yaw;
    pos_target_msg_.yaw_rate = msg.yaw_dot;

}

// looped function
void FUELInterface::controllerExecution(){
    // keep this hear to use services from base class
    
    // pose_pub_.publish(pose);
    pos_pub_.publish(pos_target_msg_);

}

bool FUELInterface::enableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    // signal FUEL planner to start.
    geometry_msgs::PoseStamped msg;
    fuel_trigger_pub_.publish(msg);
    return Controller::enableControlCallback(req, res);
}


void FUELInterface::sensorPoseTimer(const ros::TimerEvent& event){
    geometry_msgs::TransformStamped tfGeom;
    try {
        tfGeom = buffer.lookupTransform("map", "camera_link", ros::Time(0), ros::Duration(4.0));
    } catch (tf2::TransformException &e) {
        ROS_ERROR("Cant find transform");
    }
    geometry_msgs::PoseStamped msg;
    msg.header = tfGeom.header;
    msg.pose.position.x = tfGeom.transform.translation.x;
    msg.pose.position.y = tfGeom.transform.translation.y;
    msg.pose.position.z = tfGeom.transform.translation.z;
    
    msg.pose.orientation = tfGeom.transform.rotation;

    sensor_pose_pub.publish(msg);   
}