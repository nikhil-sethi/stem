
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
// #include /

sensor_msgs::PointCloud2 cloud_msg;
ros::Publisher pcl_pub;

float norm(pcl::PointXYZRGB point){
    return std::sqrt(pow(point.x,2) + pow(point.y,2) +pow(point.z,2));
}

float bgr_to_rgb(float bgr){
    // unpack rgb into r/g/b
    std::uint32_t bgr_int = *reinterpret_cast<int*>(&bgr);
    std::uint8_t b = (bgr_int >> 16) & 0x0000ff;
    std::uint8_t g = (bgr_int >> 8)  & 0x0000ff;
    std::uint8_t r = (bgr_int)       & 0x0000ff;

    std::uint32_t rgb_int = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);

    float rgb_float = *reinterpret_cast<float*>(&rgb_int);

    return rgb_float;
}

void repubPclTimer(const ros::TimerEvent& e){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud_msg, *pcl_cloud);
    
    // filter to save memory
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (pcl_cloud);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*pcl_cloud);
    
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (pcl_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-5.0, 1);
    //pass.setNegative (true);
    pass.filter (*pcl_cloud);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    // reorder global cloud's rgb channels
    for (auto& point: pcl_cloud->points){
        if (norm(point)>3)
            continue;
        point.rgb = bgr_to_rgb(point.rgb);
        pcl_cloud_filtered->push_back(point);
    }

    // republish
    sensor_msgs::PointCloud2 repub_msg;
    pcl::toROSMsg(*pcl_cloud_filtered, repub_msg);
    repub_msg.header = cloud_msg.header;
    // repub_msg.header.frame_id = "firefly/vi_sensor/camera_depth_optical_center_link"; // uncommend this for vsep
    repub_msg.height = pcl_cloud_filtered->height;
    repub_msg.width = pcl_cloud_filtered->width;
    // repub_msg.is_dense = cloud_msg.is_dense;

    pcl_pub.publish(repub_msg);

}

void pcl_callback(const sensor_msgs::PointCloud2 msg){
    cloud_msg = msg;   
}


int main(int argc, char** argv){
    ros::init(argc, argv, "repub_node");
    ros::NodeHandle nh;
    
    ros::Subscriber pcl_sub = nh.subscribe("/camera/depth/points", 10, pcl_callback);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/points/throttled", 10);

    ros::Timer repub_timer = nh.createTimer(ros::Duration(1), repubPclTimer);

    ros::spin();

    return 0;
}