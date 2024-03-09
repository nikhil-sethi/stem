#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
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
#include <plan_env/sdf_map.h>
#include <memory>
#include <sensor_model/camera.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <common/io.h>

using namespace Eigen;
struct Object{
    int id;
    pcl::PointCloud<pcl::PointXYZI> points;
    float max_gain;
    Eigen::Vector3d bbox_min_;
    Eigen::Vector3d bbox_max_;
    Eigen::Vector3d centroid_;
    vector<Eigen::Vector3d> viewpoint_candidates;
    std::vector<Eigen::Vector3d> projection_corners;

    // initialise projection corners to fixed length
    Object():projection_corners(8){}

    void computeInfo(){
        if (points.size() == 0) return;
        pcl::PointXYZI bbox_min;
        pcl::PointXYZI bbox_max;
        pcl::getMinMax3D(points, bbox_min, bbox_max); 
        bbox_min_ = Eigen::Vector3d(bbox_min.x, bbox_min.y, bbox_min.z);
        bbox_max_ = Eigen::Vector3d(bbox_max.x, bbox_max.y, bbox_max.z);

        centroid_ = (bbox_min_ + bbox_max_)/2.0;
        
        computeBboxCorners();
    } 

    void computeBboxCorners(){
        // Only two 3d diagonals are enough to define the view
        Eigen::Vector3d size = bbox_max_ - bbox_min_;
        
        
        
        
        
        // Eigen::Vector3d bbox_max_2 = bbox_max_;
        // Eigen::Vector3d bbox_min_2 = bbox_min_;
        // bbox_max_2(0) -=  diag(0);
        // bbox_min_2(0) +=  diag(0); 
          
        // points 
        projection_corners[0] = bbox_min_;
        projection_corners[1] = bbox_min_ + Vector3d(size(0), 0, 0);
        projection_corners[2] = bbox_min_ + Vector3d(0, size(1), 0);
        projection_corners[3] = bbox_min_ + Vector3d(size(0), size(1), 0);
        
        projection_corners[4] = bbox_max_;
        projection_corners[5] = bbox_max_ - Vector3d(size(0), 0, 0);
        projection_corners[6] = bbox_max_ - Vector3d(0, size(1), 0);
        projection_corners[7] = bbox_max_ - Vector3d(size(0), size(1), 0);
    
    }
};

// class fast_planner::SDFMap;

class AttentionMap{
    friend fast_planner::SDFMap;
    public:
        AttentionMap(ros::NodeHandle& nh);
        void attCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void occCallback(const common_msgs::uint8List& msg);
        void occInflateCallback(const common_msgs::uint8List& msg);
        void findViewpoints(Object& object);
        void loopTimer(const ros::TimerEvent& event);
        bool isNearUnknown(const Eigen::Vector3d& pos);
        uint8_t getOccupancy(const Eigen::Vector3i& id);
        uint8_t getOccupancy(const Eigen::Vector3d& pos);
        void visualize(Object& object);
        bool isObjectInView(const Object& object, const Eigen::Vector3d& pos, const Eigen::Vector3d& orient);

    private:
        std::list<Object> objects;
        ros::Subscriber att_3d_sub_, occ_sub_, occ_inflate_sub_;
        ros::Publisher bbox_pub;
        ros::Timer loop_timer_;
        vector<ros::Publisher> viz_pubs;
        fast_planner::PlanningVisualization viz;
        std::vector<uint8_t> occupancy_buffer_;
        std::vector<uint8_t> occupancy_inflate_buffer;
        std::unique_ptr<fast_planner::SDFMap> sdf_map_;
        double resolution_;
        double min_candidate_clearance_;

        Camera camera;
        std::vector<Eigen::Vector3d> corners_cam; //bbox corners in camera frame, just for precompute ease

};

AttentionMap::AttentionMap(ros::NodeHandle& nh):camera(nh), corners_cam(8){
    att_3d_sub_ = nh.subscribe("/attention_map/3d", 1, &AttentionMap::attCloudCallback, this);
    occ_sub_ = nh.subscribe("/occupancy_buffer", 1, &AttentionMap::occCallback, this);
    occ_inflate_sub_ = nh.subscribe("/occupancy_buffer_inflate", 1, &AttentionMap::occInflateCallback, this);
    
    bbox_pub = nh.advertise<visualization_msgs::Marker>("objects/bboxes", 1);

    // create sdf map separate instance just for function reuse
    sdf_map_.reset(new fast_planner::SDFMap);
    sdf_map_->setParams(nh, "/exploration_node/");
    
    resolution_ = sdf_map_->getResolution();
    std::cout << sdf_map_->mp_->box_mind_ << std::endl;
    min_candidate_clearance_ = nh.param("/exploration_node/frontier_finder/min_candidate_clearance", min_candidate_clearance_, -1.0);
    loop_timer_ = nh.createTimer(ros::Duration(0.05), &AttentionMap::loopTimer, this);
    viz = fast_planner::PlanningVisualization(nh);

}

void AttentionMap::visualize(Object& object){

    // === Object bounding box
    viz.drawBox((object.bbox_min_ + object.bbox_max_)/2.0, object.bbox_max_ - object.bbox_min_, Eigen::Vector4d(0.5, 0, 1, 0.3), "box"+std::to_string(object.id), object.id, 7);

    // === Enclosed point cloud with attention
    // sensor_msgs::PointCloud2 cluster_msg;
    // pcl::toROSMsg(*object.points, cluster_msg);
    // clustered_point_cloud_pub.publish(cluster_msg);

    // === Viewpoint candidates
    
    // for (auto& corner: object.projection_corners)
    //     object.viewpoint_candidates.push_back(corner); // just for debugging
    
    viz.drawSpheres(object.viewpoint_candidates, 0.2, Vector4d(0, 0.5, 0, 1), "points"+std::to_string(object.id), object.id, 6);
    // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
    // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
    // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  
}

void AttentionMap::loopTimer(const ros::TimerEvent& event){
    // for each object
        // find top viewpoints
        // calculate optimal local tour

    for (Object& object: objects){
        findViewpoints(object);
        visualize(object);
    }
    
}

void AttentionMap::findViewpoints(Object& object){
    
    // sample viewpoints around attentive regions of the object
    // for each sample 
        // evaluate information gain by ray casting (need attention buffer for this)
        // if gain is more than min gain, add viewpoint to candidate list
    // sort viewpoints by their gain
    // update top viewpoints for the object

    // 

    // Evaluate sample viewpoints on circles, find ones that cover most cells
    
    
    // find the minimum radius for cylindrical sampling
        // find the smallest face of the bbox
        // find the AR of the smallest face
        // if this AR is larger than image ar
            // bound the face in image using the width of smallest face
        // else
            // bound using the height
        // find the rmin using the bound
    Eigen::Vector3d diag_3d = object.bbox_max_ - object.bbox_min_;
    // int shortest_axis;
    // Eigen::Vector2d diag_2d_min(0.0 ,diag_3d(2));
    int shortest_axis = diag_3d(1) > diag_3d(0) ? 0: 1;
    Eigen::Vector2d diag_2d = {diag_3d(shortest_axis), diag_3d(2)};
    
    // this distance might be obsolete now that we have isometric views. but still nice starting point
    double rmin = camera.getMinDistance(diag_2d); 
    
    rmin += diag_3d(1-shortest_axis)/2; // add the longer axis to rmin because the cylinder starts at the centroid
    
    for (double rc = rmin, dr = 0.5/2; rc <= rmin + 0.5 + 1e-3; rc += dr)
        for (double phi = -M_PI; phi < M_PI; phi += 0.5235) {
            Vector3d sample_pos = object.centroid_ + rc * Vector3d(cos(phi), sin(phi), 0);
            sample_pos[2] = sample_pos[2] + 0.1; // add a height to view the object isometrically. this will depend on the data from the sensor model
            if (!sdf_map_->isInBox(sample_pos) || sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
                continue;
            // === Check if object is in view
            if (!isObjectInView(object, sample_pos, Eigen::Vector3d(0,0,phi + M_PI))){
                // print_eigen(sample_pos);
                continue;
            }
            
            // raycast from virtual camera to corners


            object.viewpoint_candidates.push_back(sample_pos);
        }

        
}       

// returns true if a full 6D viewpoint can completely view set of points
bool AttentionMap::isObjectInView(const Object& object, const Eigen::Vector3d& pos, const Eigen::Vector3d& orient){
    
    // Exstrinsic transformation (world --> viewpoint --> camera)

    // World to viewpoint
    Eigen::Isometry3d T_world_sample; // world with respect to sample
    T_world_sample.translation() = pos;
      
    Quaterniond quat;
    quat = AngleAxisd(orient(0), Vector3d::UnitX())
        * AngleAxisd(orient(1), Vector3d::UnitY())
        * AngleAxisd(orient(2), Vector3d::UnitZ());
    
    T_world_sample.linear() = quat.toRotationMatrix();

    // Viewpoint to camera
    Eigen::Isometry3d T_world_cam = T_world_sample*camera.T_odom_cam; // this transform takes a point in world frame to camera frame
    
    // Transform object corners to camera frame
    camera.transform(object.projection_corners, corners_cam, T_world_cam.inverse()); 

    // project corners to image frame and check bounds
    return camera.arePtsInView(corners_cam);
}

void AttentionMap::occCallback(const common_msgs::uint8List& msg){
    occupancy_buffer_ = msg.data;
    // std::cout<<occupancy_buffer_.size()<<std::endl;
}

void AttentionMap::occInflateCallback(const common_msgs::uint8List& msg){
    sdf_map_->md_->occupancy_buffer_inflate_ = msg.data;
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
    // ROS_INFO("num obj: %d", cluster_indices.size());
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
        object.id = ++i;
        object.points = *cluster_cloud;
        object.computeInfo();
        objects.push_back(object);
    }
}


// redefine functions for custom use

bool AttentionMap::isNearUnknown(const Eigen::Vector3d& pos) {
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (getOccupancy(vox) == fast_planner::SDFMap::UNKNOWN) return true;
      }
  return false;
}


inline uint8_t AttentionMap::getOccupancy(const Eigen::Vector3i& id) {
  if (!sdf_map_->isInMap(id)) return -1;
  return occupancy_buffer_[sdf_map_->toAddress(id)];
}

inline uint8_t AttentionMap::getOccupancy(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  sdf_map_->posToIndex(pos, id);
  return getOccupancy(id);
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