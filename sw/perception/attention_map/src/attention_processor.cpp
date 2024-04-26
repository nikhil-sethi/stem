#include <ros/ros.h>

#include <chrono>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/convolution.h>
#include <pcl/filters/convolution_3d.h>
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
#include <plan_env/raycast.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/perception_utils.h>

#include <geometry_msgs/PoseArray.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>



struct TargetViewpoint: fast_planner::Viewpoint{
    float gain_;

    TargetViewpoint(Eigen::Vector3d pos, double yaw, float gain){
        pos_ = pos;
        yaw_ = yaw;
        gain_ = gain;
    }

    Eigen::Vector4d poseToEigen(){
        return Eigen::Vector4d(pos_(0), pos_(1),pos_(2), yaw_);
    }
    Eigen::Vector3d posToEigen(){
        return Eigen::Vector3d(pos_(0), pos_(1),pos_(2));
    }

    geometry_msgs::Pose toMsg(){
        geometry_msgs::Pose msg;
        // msg.header.stamp = ros::Time::now();
        
        msg.position.x = pos_(0);
        msg.position.y = pos_(1);
        msg.position.z = pos_(2);
        
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0,0,yaw_);
        quat_tf = quat_tf.normalize();
        msg.orientation = tf2::toMsg(quat_tf);
    
        return msg;
    }   

};



using namespace Eigen;
struct Object{
    int id;
    pcl::PointCloud<pcl::PointXYZI> points;
    float max_gain;
    Eigen::Vector3d bbox_min_;
    Eigen::Vector3d bbox_max_;
    Eigen::Vector3d centroid_;
    vector<TargetViewpoint> viewpoint_candidates;
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
        Eigen::Vector3d size = bbox_max_ - bbox_min_;
        
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

bool isPtInBox(const Eigen::Vector3d& point, const Eigen::Vector3d& bbox_max, const Eigen::Vector3d& bbox_min){
    for (int i=0; i<3; ++i){
        if (point(i) < bbox_min(i) || point(i) > bbox_max(i))
            return false;
    }
    return true;
}
// Function to calculate the 3D Gaussian kernel weights
std::vector<std::vector<std::vector<double>>> calculateGaussianWeights(int size, double sigma) {
    std::vector<std::vector<std::vector<double>>> weights(size, std::vector<std::vector<double>>(size, std::vector<double>(size, 0.0)));

    double normalization = 0.0;
    for (int i = -size / 2; i <= size / 2; ++i) {
        for (int j = -size / 2; j <= size / 2; ++j) {
            for (int k = -size / 2; k <= size / 2; ++k) {
                double weight = exp(-(i * i + j * j + k * k) / (2 * sigma * sigma));
                weights[i + size / 2][j + size / 2][k + size / 2] = weight;
                normalization += weight;
            }
        }
    }

    // Normalize the weights
    // for (int i = 0; i < size; ++i) {
    //     for (int j = 0; j < size; ++j) {
    //         for (int k = 0; k < size; ++k) {
    //             weights[i][j][k] /= normalization;
    //         }
    //     }
    // }

    return weights;
}
// inline vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel) {
//   vector<Eigen::Vector3i> neighbors(26);
//   Eigen::Vector3i tmp;
//   int count = 0;
//   for (int x = -1; x <= 1; ++x)
//     for (int y = -1; y <= 1; ++y)
//       for (int z = -1; z <= 1; ++z) {
//         if (x == 0 && y == 0 && z == 0) continue;
//         tmp = voxel + Eigen::Vector3i(x, y, z);
//         neighbors[count++] = tmp;
//       }
//   return neighbors;
// }
// class fast_planner::SDFMap;


inline vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel, int depth) {
  vector<Eigen::Vector3i> neighbors(pow(2*depth+1, 3)-1);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -depth; x <= depth; ++x)
    for (int y = -depth; y <= depth; ++y)
      for (int z = -depth; z <= depth; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

class AttentionMap{
    friend fast_planner::SDFMap;
    public:
        AttentionMap(ros::NodeHandle& nh);
        void attCloudCallback(const sensor_msgs::PointCloud2& msg);
        void occCallback(const common_msgs::uint8List& msg);
        void occInflateCallback(const common_msgs::uint8List& msg);
        void findViewpoints(Object& object);
        void loopTimer(const ros::TimerEvent& event);
        bool isNearUnknown(const Eigen::Vector3d& pos);
        uint8_t getOccupancy(const Eigen::Vector3i& id);
        uint8_t getOccupancy(const Eigen::Vector3d& pos);
        void visualize(Object& object);
        bool isObjectInView(const Object& object, const Eigen::Vector3d& pos, const Eigen::Vector3d& orient);
        float computeInformationGain(Object& object, const Eigen::Vector3d& sample_pos, const double& yaw);
        void createObjects();
        void publishAttTimer(const ros::TimerEvent& e);
        void filterAttBuffer();
        void diffuseAttention(Eigen::Vector3i& cell, float attention, int depth);
        bool knownfree(const Eigen::Vector3i& idx) ;
        bool isNeighborUnknown(const Eigen::Vector3i& voxel) ;
        bool isNeighborFree(const Eigen::Vector3i& voxel);
        bool isNeighborAttentive(const Eigen::Vector3i& voxel);
        
        bool isFrontier(Eigen::Vector3i idx);
        void diffusionTimer(const ros::TimerEvent& e);

    private:
        std::list<Object> objects;
        ros::Subscriber att_3d_sub_, occ_sub_, occ_inflate_sub_;
        ros::Publisher bbox_pub, vpt_pub, att_3d_pub, att_diff_pub;
        ros::Timer loop_timer_, diffusion_timer_, att_pub_timer;
        vector<ros::Publisher> viz_pubs;
        fast_planner::PlanningVisualization viz;
        std::vector<uint8_t> occupancy_buffer_;
        std::vector<uint8_t> occupancy_inflate_buffer;
        std::vector<float> attention_buffer;

        std::unique_ptr<fast_planner::SDFMap> sdf_map_;
        double resolution_;
        double min_candidate_clearance_;

        Camera camera;
        std::vector<Eigen::Vector3d> corners_cam; //bbox corners in camera frame, just for precompute ease

        std::unique_ptr<RayCaster> raycaster_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_att_cloud;

        std::vector<bool> checked;
        std::vector<std::vector<std::vector<double>>> weights;
        double kernel_size;
        float att_min, diffusion_factor;
        fast_planner::PerceptionUtils percep_utils_;

};

AttentionMap::AttentionMap(ros::NodeHandle& nh):camera(nh), corners_cam(8), percep_utils_(nh){
    att_3d_sub_ = nh.subscribe("/attention_map/local", 1, &AttentionMap::attCloudCallback, this);
    occ_sub_ = nh.subscribe("/occupancy_buffer", 1, &AttentionMap::occCallback, this);
    occ_inflate_sub_ = nh.subscribe("/occupancy_buffer_inflate", 1, &AttentionMap::occInflateCallback, this);
    // frontier_sub = = nh.subscribe("/frontier_cells", 1, &AttentionMap::frontierCallback, this);

    bbox_pub = nh.advertise<visualization_msgs::Marker>("objects/bboxes", 1);
    vpt_pub = nh.advertise<geometry_msgs::PoseArray>("/objects/target_vpts", 1);
    att_3d_pub = nh.advertise<sensor_msgs::PointCloud2>("/attention_map/global", 1);
    att_diff_pub = nh.advertise<sensor_msgs::PointCloud2>("/attention_map/blur", 10);

    // create sdf map separate instance just for function reuse
    sdf_map_.reset(new fast_planner::SDFMap);
    sdf_map_->setParams(nh, "/exploration_node/");
    resolution_ = sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    sdf_map_->getRegion(origin, size);
    print_eigen(origin);
    print_eigen(size);
    // preallocate memory for buffers
    occupancy_buffer_ = vector<uint8_t>(sdf_map_->buffer_size, 1);
    occupancy_inflate_buffer = vector<uint8_t>(sdf_map_->buffer_size, 0);
    attention_buffer = vector<float>(sdf_map_->buffer_size, 0);

    checked = vector<bool>(sdf_map_->buffer_size, false);

    min_candidate_clearance_ = nh.param("/exploration_node/frontier_finder/min_candidate_clearance", min_candidate_clearance_, -1.0);
    loop_timer_ = nh.createTimer(ros::Duration(0.05), &AttentionMap::loopTimer, this);
    att_pub_timer =  nh.createTimer(ros::Duration(0.05), &AttentionMap::publishAttTimer, this);
    diffusion_timer_ = nh.createTimer(ros::Duration(0.05), &AttentionMap::diffusionTimer, this);
    

    // visualization
    viz = fast_planner::PlanningVisualization(nh);

    // raycasting
    raycaster_.reset(new RayCaster);
    raycaster_->setParams(resolution_, origin);
    // percep_utils_ = fast_planner::PerceptionUtils(nh);

    global_att_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    global_att_cloud->height = 1;
    global_att_cloud->is_dense = true;
    global_att_cloud->header.frame_id = "map";
    kernel_size = 5;
    weights = calculateGaussianWeights(kernel_size, 2); // precompute weights for kernel


    nh.param("/perception/attention_map/3d/att_min", att_min, 0.1f); 
    nh.param("/perception/attention_map/3d/diffusion_factor", diffusion_factor, 0.9f); 


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
    
    if (!object.viewpoint_candidates.empty()){
        std::vector<Eigen::Vector3d> vpt_positions;
        // plotting only the best for now
        // vpt_positions.push_back(object.viewpoint_candidates[0].posToEigen());

        // plotting top K viewpoints
        int num_vpts = std::min((int)object.viewpoint_candidates.size(), 3);
        for (int i = 0; i < num_vpts; i++){
            vpt_positions.push_back(object.viewpoint_candidates[i].posToEigen());
        
        }
         
            


        // for (auto& vpt: object.viewpoint_candidates){
        //     vpt_positions.push_back(Eigen::Vector3d(vpt.pos_(0), vpt.pos_(1), vpt.pos_(2)));
        // }

        viz.drawSpheres(vpt_positions, 0.2, Vector4d(0, 0.5, 0, 1), "points"+std::to_string(object.id), object.id, 6);
        // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
        // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
        // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  
    }
    
}

void AttentionMap::loopTimer(const ros::TimerEvent& event){
    // for each object
        // find top viewpoints
        // calculate optimal local tour

    geometry_msgs::PoseArray vpts_msg;
    vpts_msg.header.stamp = ros::Time::now();
    vpts_msg.header.frame_id = "map";
    // std::vector<geometry_msgs::Pose> vpts(object.size());


    createObjects(); // cluster the global point cloud 
    
    for (Object& object: objects){
        // if (object.id==3){
        findViewpoints(object);
        visualize(object);
        if (!object.viewpoint_candidates.empty()){
            int num_vpts = std::min((int)object.viewpoint_candidates.size(), 3);
            // int num_vpts =1;
            for (int i=0; i<num_vpts; i++)
                vpts_msg.poses.push_back(object.viewpoint_candidates[i].toMsg());
        }
    }

    vpt_pub.publish(vpts_msg); // publish poses for use by lkh tsp inside fuel


}

void sortViewpoints(std::vector<TargetViewpoint>& vpts){
    sort(vpts.begin(), vpts.end(), [](const TargetViewpoint& v1, const TargetViewpoint& v2) { return v1.gain_ > v2.gain_; });   
}


void AttentionMap::findViewpoints(Object& object){
    
    // find the closest distance that would have the object still in view
    // sample viewpoints around the object starting from the minimum distance
    // for each sample 
        // if sample is near unknown region (OR) on an occupied cell (OR) not in the volume --> discard
        // if the object's perspective view from the sample doesnt fit in the camera --> discard 
        // evaluate information gain by ray casting (need attention buffer for this)
        // if gain is more than min gain, add viewpoint to candidate list
    // sort viewpoints by their gain
    // get some top viewpoints for the object



    // find the minimum radius for cylindrical sampling
        // find the smallest face of the bbox
        // find the AR of the smallest face
        // if this AR is larger than image ar
            // bound the face in image using the width of smallest face
        // else
            // bound using the height
        // find the rmin using the bound
    Eigen::Vector3d diag_3d = object.bbox_max_ - object.bbox_min_;
    int shortest_axis = diag_3d(1) > diag_3d(0) ? 0: 1;
    Eigen::Vector2d diag_2d = {diag_3d(shortest_axis), diag_3d(2)};
    
    // this distance might be obsolete now that we have isometric views. but still nice starting point
    double rmin = camera.getMinDistance(diag_2d); 

    rmin += diag_3d(1-shortest_axis)/2; // add the longer axis to rmin because the cylinder starts at the centroid
    
    object.viewpoint_candidates.clear();
    for (double rc = rmin, dr = 0.5/2; rc <= rmin + 0.5 + 1e-3; rc += dr)
        for (double phi = -M_PI; phi < M_PI-0.5235; phi += 0.5235) {
            Vector3d sample_pos = object.centroid_ + rc * Vector3d(cos(phi), sin(phi), 0);
            sample_pos[2] = sample_pos[2] + 0.1; // add a height to view the object isometrically. this will depend on the data from the sensor model

            if (!sdf_map_->isInBox(sample_pos) || sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
                continue;

            // === Check if object is in view            
            if (!isObjectInView(object, sample_pos, Eigen::Vector3d(0,0,phi + M_PI))){
                // print_eigen(sample_pos);
                continue;
            }
            
            // raycast from virtual camera to corners. not implemented. might not need

            // compute information gain from remaining viewpoints
            float gain = computeInformationGain(object, sample_pos, phi + M_PI);
            print(object.id, rc, phi, gain);
            if (gain<=5)
                continue;
            
            // add whatever's left to candidates
            TargetViewpoint vpt(sample_pos, phi + M_PI, gain);
            object.viewpoint_candidates.push_back(vpt);
        }
        
        if (!object.viewpoint_candidates.empty())
            // sort list wrt information gain 
            sortViewpoints(object.viewpoint_candidates);       

}       

// nonlinear transfer function that modifies the gain that you see based on it's value
float infoTransfer(float gain){
    int alpha = 4; // higher if you want more local object region importance over object coverage. But small low attention objects might get missed out then
    return 1 + pow(alpha*(gain-0.5), 3);
}

float AttentionMap::computeInformationGain(Object& object, const Eigen::Vector3d& sample_pos, const double& yaw){
    
    Eigen::Vector3i idx;
    Eigen::Vector3i start_idx;
    Eigen::Vector3d pos;
    float total_gain = 0;
    percep_utils_.setPose(sample_pos, yaw);

    //inflated bounding box around object bbox
    Eigen::Vector3d bbox_min = object.bbox_min_ - Eigen::Vector3d(0.5,0.5,0.5);
    Eigen::Vector3d bbox_max = object.bbox_max_ + Eigen::Vector3d(0.5,0.5,0.5);

    for (pcl::PointXYZI& point : global_att_cloud->points) {
        pos(0) = point.x;
        pos(1) = point.y;
        pos(2) = point.z;
        Eigen::Vector3i idx_;
        sdf_map_->posToIndex(pos, idx_);
        if (!getOccupancy(idx_) == fast_planner::SDFMap::UNKNOWN || !percep_utils_.insideFOV(pos)) continue;
        
        raycaster_->input(pos, sample_pos);
        bool visib = true;
        // sdf_map_->posToIndex(pos, start_idx); // save start to cleanup later 
        raycaster_->nextId(idx); // because we're already on the surface, presumably
        start_idx = idx;
        while (raycaster_->nextId(idx)) {
            if (
                attention_buffer[sdf_map_->toAddress(idx)] > 0 // the ray shouldnt have any other attentive cell in it's path
                // sdf_map_->getInflateOccupancy(idx) == 1 ||   // not using inflation for now becauase most attentive cells will be missed out then
                || getOccupancy(idx) == fast_planner::SDFMap::OCCUPIED 
                || getOccupancy(idx) == fast_planner::SDFMap::UNKNOWN
                ) {
                    visib = false;
                    // attention_buffer[sdf_map_->toAddress(start_idx)] = 0; // mark this attention cell dead, because it's unreachable.
                    break;
            }
        }
        // if (visib) total_gain += infoTransfer(point.intensity)*point.intensity;
        if (visib) total_gain += point.intensity;
    }
    return total_gain; 
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
    // sdfmap buffer isnt used here because of different compatible uint8 datatype
    occupancy_buffer_ = msg.data;
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // pcl::PointXYZ pt;
    // Eigen::Vector3d pos;
    // Eigen::Vector3i idx;
    // for (int i=0; i<occupancy_buffer_.size(); i++){
        
    //     sdf_map_->indexToPos(i, pos);
    //     sdf_map_->posToIndex(pos, idx);
    //     if (getOccupancy(idx) == fast_planner::SDFMap::UNKNOWN){
    //         pt.x = pos(0);
    //         pt.y = pos(1);
    //         pt.z = pos(2);
    //         cloud.push_back(pt);
    //     }
    // }
    // cloud.width = cloud.points.size();
    // cloud.height = 1;
    // cloud.is_dense = true;
    // cloud.header.frame_id = "map";
    // sensor_msgs::PointCloud2 cloud_msg;
    // pcl::toROSMsg(cloud, cloud_msg);
    // unknown_pub_.publish(cloud_msg);
}

void AttentionMap::occInflateCallback(const common_msgs::uint8List& msg){
    sdf_map_->md_->occupancy_buffer_inflate_ = msg.data;
}


void AttentionMap::filterAttBuffer(){
    global_att_cloud->points.clear();
    pcl::PointXYZI pcl_pt;
    Eigen::Vector3i idx;
    Eigen::Vector3d pos;
    std::vector<int> salient_voxels_ids;
    for (int i = 0; i<attention_buffer.size(); i++){
        if (attention_buffer[i]<=0) // only check positive attention cells
            continue;
        
        // if (attention_buffer[i]!=1){ // dont do anything for salient voxels except adding them to the cloud
            sdf_map_->indexToPos(i, pos);
            sdf_map_->posToIndex(pos, idx);
        ROS_ERROR("%d", getOccupancy(idx));
            //  if (getOccupancy(idx) == fast_planner::SDFMap::UNKNOWN){
            //         attention_buffer[i] = 1;
            //     }
            // auto nbrs = allNeighbors(idx); // 26 neighbors
            // float att_nbr = 0.0;
            
            // for (auto nbr : nbrs) {
            //     if (!sdf_map_->isInMap(nbr))
            //         continue;
            //     // int adr = sdf_map_->toAddress(nbr);
            //     // att_nbr += attention_buffer[adr];
            //     if (getOccupancy(nbr) == fast_planner::SDFMap::UNKNOWN){
            //         pcl_pt.x = pos[0];
            //         pcl_pt.y = pos[1];
            //         pcl_pt.z = pos[2];
            //         pcl_pt.intensity = 1;
            //         global_att_cloud->push_back(pcl_pt);
            //     }
                        
            // }
            
            // if ((att_nbr < 3*0.5) // at least 4 attentive neighbours
            //     && (getOccupancy(idx) ==fast_planner::SDFMap::OCCUPIED))
            //     attention_buffer[i] = 0;    
        
        // }

        // add to global cloud
        // sdf_map_->indexToPos(i, pos);
        
    }

    // for (auto id: salient_voxels_ids){
    //     attention_buffer[id] = 1;
    // }
}

bool enforceIntensitySimilarity (const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
{
  if (squared_distance < 1 && point_a.intensity>1.9 && std::abs (point_a.intensity - point_b.intensity) < 1e-2f)
    return (true);
  else
    return (false);
}


void AttentionMap::createObjects(){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>(*global_att_cloud));

    // Apply clustering
    std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    // ec.setInputCloud(cloud_copy);
    // ec.setClusterTolerance(0.15); // Set the cluster tolerance (adjust as needed)
    // ec.setMinClusterSize(5);    // Set the minimum cluster size (adjust as needed)
    // ec.setMaxClusterSize(25000);   // (neSet the maximum cluster size (adjust as needed)
    // ec.extract(cluster_indices);


    pcl::ConditionalEuclideanClustering<pcl::PointXYZI> cec (true);
    cec.setInputCloud (cloud_copy);
    cec.setConditionFunction (&enforceIntensitySimilarity);
    cec.setClusterTolerance (0.15);
    cec.setMinClusterSize (5);
    cec.setMaxClusterSize (100);
    cec.segment (cluster_indices);
    // cec.getRemovedClusters (small_clusters, large_clusters);



    // ROS_INFO("num obj: %d", cluster_indices.size());
    objects.clear();
    int i = 0;
    for (const auto& cluster_index : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointIndices::Ptr pcl_indices(new pcl::PointIndices(cluster_index));
        extract.setInputCloud(cloud_copy);
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

void AttentionMap::publishAttTimer(const ros::TimerEvent& e){
    Eigen::Vector3d pos;
    // Create PCL cloud
    global_att_cloud->points.clear();
    pcl::PointXYZI pcl_pt;
    for (int i=0; i<attention_buffer.size(); i++){
        float attention = attention_buffer[i];

        if (attention<=0.0f) // this is the main thing that saves compute
            continue;

        sdf_map_->indexToPos(i, pos);
        pcl_pt.x = pos[0];
        pcl_pt.y = pos[1];
        pcl_pt.z = pos[2];
        pcl_pt.intensity = attention;
        global_att_cloud->push_back(pcl_pt);  

    }

    global_att_cloud->width = global_att_cloud->points.size();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*global_att_cloud, cloud_msg);
    att_3d_pub.publish(cloud_msg);
}

vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(6);
  Eigen::Vector3i tmp;

  tmp = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[0] = tmp;
  tmp = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[1] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[2] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[3] = tmp;
  tmp = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[4] = tmp;
  tmp = voxel + Eigen::Vector3i(0, 0, 1);
  neighbors[5] = tmp;

  return neighbors;
}

vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}


bool AttentionMap::knownfree(const Eigen::Vector3i& idx) {
  return getOccupancy(idx) == fast_planner::SDFMap::FREE;
}

bool AttentionMap::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (getOccupancy(nbr) == fast_planner::SDFMap::UNKNOWN) return true;
  }
  return false;
}

bool AttentionMap::isNeighborFree(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (getOccupancy(nbr) == fast_planner::SDFMap::FREE) return true;
  }
  return false;
}

bool AttentionMap::isNeighborAttentive(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = allNeighbors(voxel, 1);
  for (auto nbr : nbrs) {
    if (attention_buffer[sdf_map_->toAddress(nbr)]>0) return true;
  }
  return false;
}


bool AttentionMap::isFrontier(Eigen::Vector3i idx){
    // return knownfree(idx) && isNeighborUnknown(idx);
    return getOccupancy(idx)== fast_planner::SDFMap::UNKNOWN && isNeighborFree(idx);
}


void gaussian_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered,
                    double radius)
{
/*double radius = 0.02;*/
// Set up the Gaussian Kernel
pcl::filters::GaussianKernel<pcl::PointXYZI, pcl::PointXYZI>::Ptr kernel(
    new pcl::filters::GaussianKernel<pcl::PointXYZI, pcl::PointXYZI>);
(*kernel).setSigma(4);
(*kernel).setThresholdRelativeToSigma(4);
std::cout << "Kernel made" << std::endl;

// Set up the KDTree
pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(
    new pcl::search::KdTree<pcl::PointXYZI>);
(*kdtree).setInputCloud(cloud);
std::cout << "KdTree made" << std::endl;

// Set up the Convolution Filter
pcl::filters::Convolution3D<
    pcl::PointXYZI,
    pcl::PointXYZI,
    pcl::filters::GaussianKernel<pcl::PointXYZI, pcl::PointXYZI>>
    convolution;
convolution.setKernel(*kernel);
convolution.setInputCloud(cloud);
convolution.setSearchMethod(kdtree);
convolution.setRadiusSearch(radius);
convolution.setNumberOfThreads(
    10); // important! Set Thread number for openMP
std::cout << "Convolution Start" << std::endl;
convolution.convolve(*cloud_filtered);
std::cout << "Convoluted" << std::endl;
}


void AttentionMap::attCloudCallback(const sensor_msgs::PointCloud2& msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_att_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *local_att_cloud);


    // ====== Density filtering =======

    // Remove radius outliers in the local cloud before adding to the global one
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_att_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    
    if (local_att_cloud->size()>0){
        outrem.setInputCloud(local_att_cloud);
        outrem.setRadiusSearch(0.2);
        outrem.setMinNeighborsInRadius (3);
        outrem.setKeepOrganized(true);
        outrem.filter (*local_att_cloud_filtered);

    }
    
    // ======= Global map update ======

    // update the global buffer. Very small loop
    float alpha = 1;
    Eigen::Vector3d pos;
    Eigen::Vector3i idx;
    for (auto& pt: local_att_cloud_filtered->points){
        float attention = pt.intensity;
        pos[0] = pt.x;
        pos[1] = pt.y;
        pos[2] = pt.z;
        sdf_map_->posToIndex(pos, idx);
        // attention_buffer[msg.indices[i]] = attention;
        if (sdf_map_->isInMap(idx)){
            int vox_adr = sdf_map_->toAddress(idx);
            if (getOccupancy(idx) == fast_planner::SDFMap::OCCUPIED){
                float att_update = alpha*attention + (1-alpha)*attention_buffer[vox_adr];
                attention_buffer[vox_adr] = att_update > 0.2 ? attention: 0; // discrete for now
            }
        }
    }

    // publishAtt();
}

// Diffuse attentive semantics into neighboring frontiers. 
// Run in a separate thread because computationally expensive and we dont need fast updates.
void AttentionMap::diffusionTimer(const ros::TimerEvent& e){
    std::fill(checked.begin(), checked.end(), false);
    Eigen::Vector3i idx;
    Eigen::Vector3d pos;
    
    for (int i=0; i<attention_buffer.size(); i++){
        float attention = attention_buffer[i];

        if (attention<=att_min){ // this is the main thing that saves compute
            attention_buffer[i]=0;
            continue;
        }

        sdf_map_->indexToPos(i, pos);
        sdf_map_->posToIndex(pos, idx);

        auto nbrs = tenNeighbors(idx); // 10 neighbors
        float att_nbr = 0.0; 
        
        Eigen::Vector3d nbr_pos; // need a new pos var so the att map doesn't get overriden
        for (auto nbr : nbrs) {
            sdf_map_->indexToPos(nbr, nbr_pos);
            if (nbr_pos(2)<0.2 || !sdf_map_->isInMap(nbr))
                continue;
                
            // calculate nearby attention
            int nbr_adr = sdf_map_->toAddress(nbr);
            // att_nbr += attention_buffer[nbr_adr] >0 ? attention_buffer[nbr_adr]: 0;
            
            // save list of unknown voxels near attentive regions
            if (!checked[nbr_adr] && isFrontier(nbr)){
                // bu_voxels.push_back(nbr_adr);

                auto nbrs2 = allNeighbors(idx, 1); // 26 neighbors
                float att_nbr2 = 0.0; 
                float count2 = 0;
                for (auto nbr2 : nbrs2) {
                    sdf_map_->indexToPos(nbr2, nbr_pos);
                    int nbr2_adr = sdf_map_->toAddress(nbr2);
                    if (attention_buffer[nbr2_adr] <att_min || nbr_pos(2)<0.2 || !sdf_map_->isInMap(nbr) )
                        continue;
                    
                    att_nbr2 += diffusion_factor*attention_buffer[nbr2_adr];
                    count2++;
                }
                float att_diffused = (count2>0)? att_nbr2/count2:0;
                
                // the weighted update just allows it to stabilise a bit, otherwise a lot of flickering
                attention_buffer[nbr_adr] = att_diffused>att_min?  (0.9*att_diffused + 0.1*attention_buffer[nbr_adr]):0;
                checked[nbr_adr] = true;
            }
        
        }

        // clean up free space. Old cells stick around if not updated anymore
        if (getOccupancy(idx) == fast_planner::SDFMap::FREE) {
            attention_buffer[i]=0;
        }

    }
}

void AttentionMap::diffuseAttention(Eigen::Vector3i& cell, float attention, int depth){
    int adr = sdf_map_->toAddress(cell);
    if (depth >7 || checked[adr] || !isFrontier(cell)){
    // (occupancy_buffer_[adr] == fast_planner::SDFMap::UNKNOWN))
        return;
    }
    float att_diffused= 0.9*attention;
    // if (depth >5) {
    //     att_diffused = 1;
    // }
    // else{
    //     att_diffused = 0.9*attention; // exponential decrease
    // }

    int this_depth = depth + 1;
    // Eigen::Vector3d pos; 
    // Eigen::Vector3i idx; 
    // sdf_map_->indexToPos(adr, pos);
    // sdf_map_->posToIndex(pos, idx);
    auto nbrs = allNeighbors(cell, 2);

    Eigen::Vector3d nbr_pos; // need a new pos var so the att map doesn't get overriden
    // std::cout<<this_depth<<std::endl;
    // print(this_depth);
    double sum = 0;
    double count = 0;
    for (auto& nbr: nbrs){

        sdf_map_->indexToPos(nbr, nbr_pos);
        int nbr_adr = sdf_map_->toAddress(nbr);
        float att_nbr = attention_buffer[nbr_adr];
        if (!sdf_map_->isInMap(nbr) || nbr_pos(2)<0.2)
            continue;
            
        sum += att_nbr > 0? 0.9*att_nbr:0;
        count += att_nbr > 0? 1: 0;
        diffuseAttention(nbr, 0.9*att_nbr, this_depth); // redo the thing for each neighbor until max depth

    }

    attention_buffer[adr] = sum/count;
    checked[adr] = true; // mark checked to prevent revisit. saves a lot of compute

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