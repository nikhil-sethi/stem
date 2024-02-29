#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <list>
#include <vector>
#include <Eigen/Eigen>
#include <traj_utils/planning_visualization.h>
#include <common_msgs/uint8List.h>

struct Object{
    pcl::PointCloud<pcl::PointXYZI> points;
    float max_gain;
    pcl::PointXYZI bbox_min;
    pcl::PointXYZI bbox_max;

    void computeInfo(){
        if (points.size() == 0) return;
        pcl::getMinMax3D(points, bbox_min, bbox_max); 

    }
};
class AttentionMap{
    public:
        AttentionMap(ros::NodeHandle& nh);
        void attCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void occCallback(const common_msgs::uint8List& msg);
        void findViewpoints(Object& object);
        void loop();

    private:
        std::list<Object> objects;
        ros::Subscriber att_3d_sub_, occ_sub_;
        ros::Publisher clustered_point_cloud_pub, bbox_pub;
        vector<ros::Publisher> viz_pubs;
        fast_planner::PlanningVisualization viz;
        std::vector<uint8_t> occupancy_buffer;
};

AttentionMap::AttentionMap(ros::NodeHandle& nh){
    att_3d_sub_ = nh.subscribe("/attention_map/3d", 1, &AttentionMap::attCloudCallback, this);
    occ_sub_ = nh.subscribe("/occupancy_map", 1, &AttentionMap::occCallback, this);
    clustered_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/attention_map/3d_clustered", 1);
    bbox_pub = nh.advertise<visualization_msgs::Marker>("objects/bboxes", 1);
    
    viz = fast_planner::PlanningVisualization(nh);
    
}

void AttentionMap::loop(){
    // for each object
        // find top viewpoints
        // calculate optimal local tour
}

void AttentionMap::findViewpoints(Object& object){
    
    // sample viewpoints around attentive regions of the object
    // for each sample 
        // evaluate information gain by ray casting (need attention buffer for this)
        // if gain is more than min gain, add viewpoint to candidate list
    // sort viewpoints by their gain
    // update top viewpoints for the object

    // 


}       



void AttentionMap::occCallback(const common_msgs::uint8List& msg){
    occupancy_buffer = msg.data;
}


void AttentionMap::attCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    // Convert sensor_msgs::PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Remove NaN values
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*pcl_cloud, *filtered_cloud, indices);

    // Apply clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(pcl_cloud);
    ec.setClusterTolerance(0.1); // Set the cluster tolerance (adjust as needed)
    ec.setMinClusterSize(10);    // Set the minimum cluster size (adjust as needed)
    ec.setMaxClusterSize(25000);   // Set the maximum cluster size (adjust as needed)
    ec.extract(cluster_indices);
    ROS_INFO("%d", cluster_indices.size());
    objects.clear();
    int i = 0;
    for (const auto& cluster_index : cluster_indices) {
        

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointIndices::Ptr pcl_indices(new pcl::PointIndices(cluster_index));
        extract.setInputCloud(pcl_cloud);
        extract.setIndices(pcl_indices);
        extract.filter(*cluster_cloud);
        
        // create objects from clustered clouds
        Object object;
        object.points = *cluster_cloud;
        object.computeInfo();
        objects.push_back(object);

        // sensor_msgs::PointCloud2 cluster_msg;
        // pcl::toROSMsg(*cluster_cloud, cluster_msg);
        Eigen::Vector3d bbox_min = Eigen::Vector3d(object.bbox_min.x, object.bbox_min.y, object.bbox_min.z);
        Eigen::Vector3d bbox_max = Eigen::Vector3d(object.bbox_max.x, object.bbox_max.y, object.bbox_max.z);

        // clustered_point_cloud_pub.publish(cluster_msg);
        viz.drawBox((bbox_min+bbox_max)/2.0, bbox_max - bbox_min, Eigen::Vector4d(0.5, 0, 1, 0.3), "box"+std::to_string(i), ++i, 7);
    
    }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "attention_processor_node");
  ros::NodeHandle nh("~");

  AttentionMap att_proc(nh);
//   expl_fsm.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;

}