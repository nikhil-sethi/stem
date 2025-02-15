#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include <cmath>


#define PI 3.141592
#define EARTH_RADIUS 6371000.0

class FakeEKFOriginLocalizer{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;
        ros::Publisher ekf_origin_pub_;
        geometry_msgs::Point target_pos;

        geographic_msgs::GeoPoint origin_lla;

        ros::Time last_try_time;
        ros::Duration timeout;


    public: 
        FakeEKFOriginLocalizer(ros::NodeHandle& nh){   

            ROS_INFO("Created origin setter.");
            timeout = ros::Duration(1.0);

            nh.param("init_x", target_pos.x, 0.0);
            nh.param("init_y", target_pos.y, 0.0);
            nh.param("init_z", target_pos.z, 0.0);

            // Subscribe to the odometry topic
            odom_sub_ = nh.subscribe("/mavros/local_position/odom", 10, &FakeEKFOriginLocalizer::odomCallback, this);

            // Advertise the PoseStamped topic
            ekf_origin_pub_ = nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 10);
        }

        bool poseClose(geometry_msgs::Point source, geometry_msgs::Point target){
            if (abs(source.x - target.x)< 0.1 && 
                abs(source.y - target.y)< 0.1 &&
                abs(source.z - target.z)< 0.1){
                    ROS_INFO_STREAM("x: "<<source.x <<"xt: "<< target.x);
                    ROS_INFO_STREAM("y: "<<source.y <<"yt: "<< target.y);
                    ROS_INFO_STREAM("z: "<<source.z <<"zt: "<< target.z);
                    return true;
                }
            else{
                return false;
            }
        }

        void meters_to_lla(geographic_msgs::GeoPoint& origin, const geometry_msgs::Point& delta_meters){                
            origin.longitude -= (delta_meters.x / (EARTH_RADIUS * cos(origin.latitude * PI / 180.0))) * (180.0 / PI);
            origin.latitude -= (delta_meters.y / EARTH_RADIUS) * (180.0 / PI);
            origin.altitude -= delta_meters.z; 
            origin.altitude += 47.221; // requires some ellipoid correection idfk
            ROS_INFO_STREAM("lon: "<<origin.longitude <<"lat: "<< origin.latitude);
        }


        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            if (!poseClose(msg->pose.pose.position, target_pos)){
                
                if ((ros::Time::now()-last_try_time) > timeout){
                    ROS_INFO("Setting origin..");
                    geographic_msgs::GeoPointStamped gp_origin_msg; 

                    gp_origin_msg.header = msg->header;

                    // the actual origin 0,0,0
                    // found by checking the vehicle_local_position uorb topic after startup
                    gp_origin_msg.position.latitude = 47.397733;
                    gp_origin_msg.position.longitude = 8.545567;
                    gp_origin_msg.position.altitude = 488;// 535.221;

                    meters_to_lla(gp_origin_msg.position, target_pos);

                    ekf_origin_pub_.publish(gp_origin_msg);
                    last_try_time = ros::Time::now();
                }
            }
            else{
                ROS_INFO("Pose close enough. exiting. ");
                ros::shutdown();

            }
        }

};


class FakeVisionLocalizer
{
public:
    FakeVisionLocalizer(ros::NodeHandle& nh)
    {
        // Subscribe to the odometry topic
        odom_sub_ = nh.subscribe("/gazebo/ground_truth/odom", 10, &FakeVisionLocalizer::odomCallback, this);

        // Advertise the PoseStamped topic
        pose_stamped_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Create a PoseStamped message
        geometry_msgs::PoseStamped pose_stamped_msg;

        // Fill in the header
        pose_stamped_msg.header = msg->header;
        pose_stamped_msg.header.stamp = ros::Time::now();
        // Copy the pose from the odometry message
        pose_stamped_msg.pose = msg->pose.pose;

        // Publish the PoseStamped message
        pose_stamped_pub_.publish(pose_stamped_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher pose_stamped_pub_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "fake_localization_node");
    ros::NodeHandle nh("~");
    // Create an instance of the OdometryToPoseStamped class
    FakeVisionLocalizer localizer(nh);

    // Spin to process callbacks
    ros::spin();

    return 0;
}
