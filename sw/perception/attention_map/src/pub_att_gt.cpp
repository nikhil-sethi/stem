#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>



#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudPublisher
{
public:
    PointCloudPublisher() : nh_("~")
    {
        // Load PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZI>("/root/thesis_ws/src/thesis/sw/perception/attention_map/src/att_cloud_gt.pcd", cloud_) == -1)
        {
            ROS_ERROR("Couldn't read file cloud.pcd");
            return;
        }

        // Create PointCloud2 message
        pcl::toROSMsg(cloud_, cloud_msg_);
        cloud_msg_.header.frame_id = "map"; // Set your desired frame ID

        // Create the publisher
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/attention_map/3d", 1);

        // Create a timer for publishing
        timer_ = nh_.createTimer(ros::Duration(0.05), &PointCloudPublisher::timerCallback, this);
    }

private:
    void timerCallback(const ros::TimerEvent &event)
    {
        // Publish the point cloud
        pub_.publish(cloud_msg_);
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Timer timer_;

    pcl::PointCloud<pcl::PointXYZI> cloud_;
    sensor_msgs::PointCloud2 cloud_msg_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_publish_node");

    PointCloudPublisher pointCloudPublisher;

    // Keep the node running
    ros::spin();

    return 0;
}
