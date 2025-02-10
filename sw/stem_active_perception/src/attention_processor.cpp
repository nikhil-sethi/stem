#include <ros/ros.h>
// #define BOOST_BIND_NO_PLACEHOLDERS
#include <functional>

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
#include <stem_msgs/uint8List.h>
#include <plan_env/sdf_map.h>
#include <memory>
#include <stem_utils/camera.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <stem_utils/io.h>
#include <stem_utils/utils.h>
#include <plan_env/raycast.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/perception_utils.h>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <geometry_msgs/PoseArray.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <std_srvs/Trigger.h>
#include <stem_msgs/target_viewpoint.h>
#include <stem_msgs/Viewpoints.h>


Eigen::Vector3d origin(-5.5, -5.5, 0);
typedef std::chrono::time_point<std::chrono::high_resolution_clock> time_point;
typedef std::chrono::duration<double> time_diff;

using namespace Eigen;
struct Object{
    int id;
    // pcl::PointCloud<pcl::PointXYZI> points;
    float max_gain;
    Eigen::Vector3d bbox_min_;
    Eigen::Vector3d bbox_max_;
    Eigen::Vector3d centroid_;
    Eigen::Vector3d scale_;
    vector<TargetViewpoint> viewpoint_candidates;
    std::vector<Eigen::Vector3d> projection_corners;
    bool seen_ = false;
    float time_since_vpt = 1000;

    // initialise projection corners to fixed length
    Object():projection_corners(8){}
    Object(const pcl::PointCloud<pcl::PointXYZI>& points):projection_corners(8){
        if (points.size() == 0) return;
        pcl::PointXYZI bbox_min;
        pcl::PointXYZI bbox_max;
        pcl::getMinMax3D(points, bbox_min, bbox_max); 

        bbox_min_ = Eigen::Vector3d(bbox_min.x-0.02, bbox_min.y-0.02, bbox_min.z-0.02);
        bbox_max_ = Eigen::Vector3d(bbox_max.x+0.02, bbox_max.y+0.02, bbox_max.z+0.02);

        computeInfo();
    }

    void computeInfo(){
        centroid_ = (bbox_min_ + bbox_max_)/2.0;
        scale_ = bbox_max_ - bbox_min_;
        computeBboxCorners();
    } 

    void computeBboxCorners(){
        // points 
        projection_corners[0] = bbox_min_;
        projection_corners[1] = bbox_min_ + Vector3d(scale_(0), 0, 0);
        projection_corners[2] = bbox_min_ + Vector3d(0, scale_(1), 0);
        projection_corners[3] = bbox_min_ + Vector3d(scale_(0), scale_(1), 0);
        
        projection_corners[4] = bbox_max_;
        projection_corners[5] = bbox_max_ - Vector3d(scale_(0), 0, 0);
        projection_corners[6] = bbox_max_ - Vector3d(0, scale_(1), 0);
        projection_corners[7] = bbox_max_ - Vector3d(scale_(0), scale_(1), 0);
    
    }

    // merge bbox corners
    void merge(const Object& other){
        bbox_min_ = bbox_min_.cwiseMin(other.bbox_min_);
        bbox_max_ = bbox_max_.cwiseMax(other.bbox_max_);
        computeInfo(); // update
    }

    bool isInBox(Eigen::Vector3d box_min, Eigen::Vector3d box_max){
        return isPtInBox(bbox_min_, box_min, box_max) && isPtInBox(bbox_max_, box_min, box_max);
    }

};


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
        void occCallback(const stem_msgs::uint8List& msg);
        void occInflateCallback(const stem_msgs::uint8List& msg);
        void sampleViewpoints(Object& object, std::vector<TargetViewpoint>& sampled_vpts);
        void findTopViewpoints(Object& object, std::vector<TargetViewpoint>& sampled_vpts);
        void loopTimer(const ros::TimerEvent& event);
        bool isNearUnknown(const Eigen::Vector3d& pos);
        uint8_t getOccupancy(const Eigen::Vector3i& id);
        uint8_t getOccupancy(const Eigen::Vector3d& pos);
        void visualize();
        bool isObjectInView(const Object& object, const Eigen::Vector3d& pos, const Eigen::Vector3d& orient);
        float computeInformationGain(Object& object, const Eigen::Vector3d& sample_pos, const double& yaw);
        void createObjects(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::list<Object>& objects);
        void publishAttTimer(const ros::TimerEvent& e);
        void filterAttBuffer();
        bool knownfree(const Eigen::Vector3i& idx) ;
        bool isNeighborUnknown(const Eigen::Vector3i& voxel) ;
        bool isNeighborFree(const Eigen::Vector3i& voxel);
        bool isNeighborAttentive(const Eigen::Vector3i& voxel);
        
        bool isFrontier(Eigen::Vector3i idx);
        void diffusionTimer();

        float diffuse(Eigen::Vector3i bu_voxel);
        void getNearbyFrontiers(Eigen::Vector3i idx, std::vector<Eigen::Vector3i>& frontier_nbrs);
        bool metricsCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool enforceIntensitySimilarity (const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance);
        void odometryCallback(const nav_msgs::OdometryConstPtr&  msg);
        // bool areObjectsSimilar(const Object& obj_a, const Object& obj_b, double overlapThreshold);
        void objectUpdateTimer(const ros::TimerEvent& e);

    private:
        std::list<Object> global_objects;
        ros::Subscriber att_3d_sub_, occ_sub_, occ_inflate_sub_, odom_sub_;
        ros::Publisher bbox_pub, vpt_pub, att_3d_pub, att_diff_pub;
        ros::Timer loop_timer_, diffusion_timer_, att_pub_timer, obj_timer_;
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
        float total_entropy=0.0f;
        float learning_rate;
        Eigen::Vector3d self_pos, local_box_min, local_box_max;
        std::function<bool(const pcl::PointXYZI&, const pcl::PointXYZI&, float)> condfunc;
        Eigen::Vector3d origin, size;
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_sem_cloud;
        // std::list<TargetViewpoint> all_viewpoints; // for filtering
        Eigen::Matrix<double, 4, 4> colormap;


};

AttentionMap::AttentionMap(ros::NodeHandle& nh):camera(nh), corners_cam(8), percep_utils_(nh){
    att_3d_sub_ = nh.subscribe("/attention_map/local", 1, &AttentionMap::attCloudCallback, this);
    occ_sub_ = nh.subscribe("/occupancy_buffer", 1, &AttentionMap::occCallback, this);
    occ_inflate_sub_ = nh.subscribe("/occupancy_buffer_inflate", 1, &AttentionMap::occInflateCallback, this);
    odom_sub_ = nh.subscribe("/mavros/local_position/odom", 1, &AttentionMap::odometryCallback, this);
    // frontier_sub = = nh.subscribe("/frontier_cells", 1, &AttentionMap::frontierCallback, this);

    bbox_pub = nh.advertise<visualization_msgs::Marker>("objects/bboxes", 1);
    // vpt_pub_viz = nh.advertise<geometry_msgs::PoseArray>("/objects/target_poses_viz", 1);
    vpt_pub = nh.advertise<stem_msgs::Viewpoints>("/objects/target_vpts", 1);
    att_3d_pub = nh.advertise<sensor_msgs::PointCloud2>("/attention_map/global", 1);
    // att_diff_pub = nh.advertise<sensor_msgs::PointCloud2>("/attention_map/blur", 10);

    // create sdf map separate instance just for function reuse
    sdf_map_.reset(new fast_planner::SDFMap);
    sdf_map_->setParams(nh, "/exploration_node/");
    resolution_ = sdf_map_->getResolution();
    // Eigen::Vector3d origin, size;
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
    obj_timer_ = nh.createTimer(ros::Duration(0.05), &AttentionMap::objectUpdateTimer, this);
    att_pub_timer =  nh.createTimer(ros::Duration(0.05), &AttentionMap::publishAttTimer, this);
    // diffusion_timer_ = nh.createTimer(ros::Duration(0.05), &AttentionMap::diffusionTimer, this);

    // std::thread thread(foo);
    // thread.detach();

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
    nh.param("/perception/attention_map/3d/learning_rate", learning_rate, 0.5f); 

    condfunc = std::bind(&AttentionMap::enforceIntensitySimilarity, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    local_sem_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    std::thread diffusion_thread(&AttentionMap::diffusionTimer, this);
    diffusion_thread.detach();

    // jet colormap
    colormap <<
        0 ,0,255,1, // blue
        0,255,0,1, // green
        255,255,0,1, //yellow
        255,0,0,1; // red
    colormap = colormap/255;


    std::thread viz_thread(&AttentionMap::visualize, this);
    viz_thread.detach();
}

typedef std::pair<Vector3d, Vector3d> BoundingBox;


bool checkOverlap(const BoundingBox& box1, const BoundingBox& box2) {
    for (int i = 0; i < 3; ++i) {
        if (box1.second[i] < box2.first[i] || box1.first[i] > box2.second[i])
            return false; // No overlap along this axis
    }
    return true; // Overlap along all axes
}

double intersectionVolume(const BoundingBox& box1, const BoundingBox& box2) {
    Vector3d minCorner = box1.first.cwiseMax(box2.first);
    Vector3d maxCorner = box1.second.cwiseMin(box2.second);
    Vector3d dims = (maxCorner - minCorner).cwiseAbs();// cwiseMax(Vector3d::Zero());
    return dims.prod();
}

// double overlapThreshold = 0.5; 
bool areObjectsSimilar(const Object& obj_a, const Object& obj_b, double overlapThreshold){
    // if ((obj_a.bbox_min_-obj_b.bbox_min_).norm() < 0.1 && (obj_a.bbox_max_-obj_b.bbox_max_).norm() < 0.1)
    //     return true;

    BoundingBox box1(obj_a.bbox_min_-origin, obj_a.bbox_max_-origin);
    BoundingBox box2(obj_b.bbox_min_-origin, obj_b.bbox_max_-origin);

    // if overlap check IOU
    if (checkOverlap(box1, box2)){
        double intersectionVol = intersectionVolume(box1, box2);
        double totalVol1 = (box1.second - box1.first).prod();
        double totalVol2 = (box2.second - box2.first).prod();
        double overlapRatio1 = intersectionVol / totalVol1;
        double overlapRatio2 = intersectionVol / totalVol2;
        // std::cout<<overlapRatio1 <<overlapRatio2 <<std::endl;
        // Check if overlap ratio exceeds the threshold for either bounding box
        if (overlapRatio1 > overlapThreshold || overlapRatio2 > overlapThreshold)
            return true;
    }
    // 
    // if (checkAdjacent(box1, box2)){
    //     return true;
    // }
        

    return false;
}


// merge newly detected objects into existing ones
void mergeObjects(std::list<Object>& old_objects, std::list<Object> new_objects){
    // merge new objects into old ones
    for (Object obj_n: new_objects){
        bool merged = false;
        // find something to merge into
        for (auto& obj_o: old_objects){
            if (areObjectsSimilar(obj_n, obj_o, 0.5)){
                obj_o.merge(obj_n);// 
                // std::cout<<"rg"<<std::endl;
                merged = true;
                break;
                
            }
        }
        if (!merged){   
            old_objects.push_back(obj_n);
        }
    }


    // merge old objects into old ones
    std::list<std::list<Object>::iterator> remove_iterators;
    for (auto it1 = old_objects.begin(); it1!=old_objects.end();it1++){
        for (auto it2 = std::next(it1); it2!=old_objects.end();it2++){
            if (areObjectsSimilar(*it1, *it2, 0.5)){
                it1->merge(*it2);
                remove_iterators.push_back(it2);
                break;   
            }
        }    
    }
    for (auto it: remove_iterators)
        old_objects.erase(it);

}


void AttentionMap::visualize(){
    while (true){
        // find min and max gain of viewpoints. need this for color scaling
        float min_gain = 10000, max_gain=-10000;
        for (Object& object: global_objects ){
            for (auto vpt: object.viewpoint_candidates){
                if (vpt.gain_<min_gain) min_gain = vpt.gain_;
                if (vpt.gain_>max_gain) max_gain = vpt.gain_;
            }
        }
        

        int i=0;
        // geometry_msgs::PoseArray vpts_poses_msg;

        // vpts_poses_msg.header.stamp = ros::Time::now();
        // vpts_poses_msg.header.frame_id = "map";

        std::vector<Eigen::Vector3d> vpt_positions;
        std::vector<Eigen::Vector4d> vpt_colors;
        for (Object& object: global_objects ){
            // === Object bounding box
            viz.drawBox(object.centroid_, object.scale_, Eigen::Vector4d(0.5, 0, 1, 0.3), "box"+std::to_string(i), i, 7);

            // === Enclosed point cloud with attention
            // sensor_msgs::PointCloud2 cluster_msg;
            // pcl::toROSMsg(*object.points, cluster_msg);
            // clustered_point_cloud_pub.publish(cluster_msg);

            // === Object corners
            // for (auto& corner: object.projection_corners)
            //     object.viewpoint_candidates.push_back(corner); // just for debugging
            
            // === Viewpoint candidates
            for (uint j = 0; j < std::min((int)object.viewpoint_candidates.size(), 3); j++){
                auto vpt = object.viewpoint_candidates[j];
                vpt_positions.push_back(vpt.posToEigen());
                // print(min_gain, object.viewpoint_candidates[i].gain_, max_gain);
                // vpts_msg_viz.poses.push_back(vpt.toGeometryMsg());
                vpt_colors.push_back(vpt.getColor(min_gain, max_gain, colormap));
            }
            i++;
        }
        viz.drawSpheres(vpt_positions, 0.2, vpt_colors, "top viewpoints", 1, 6);
        // vpt_pub_viz.publish(vpts_poses_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
}

void AttentionMap::objectUpdateTimer(const ros::TimerEvent& e){
     std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();

    local_sem_cloud->clear();
    Eigen::Vector3i min_cut;
    Eigen::Vector3i max_cut;
    sdf_map_->posToIndex(local_box_min, min_cut);
    sdf_map_->posToIndex(local_box_max, max_cut);

    sdf_map_->boundIndex(min_cut);
    sdf_map_->boundIndex(max_cut);

    Eigen::Vector3d pos;
    pcl::PointXYZI pcl_pt;
    
    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = sdf_map_->mp_->box_min_(2); z < sdf_map_->mp_->box_max_(2); ++z) {

            int adr = sdf_map_->toAddress(x,y,z);

            float attention = attention_buffer[adr];

            if (attention >= att_min && occupancy_buffer_[adr] == fast_planner::SDFMap::OCCUPIED) {
                sdf_map_->indexToPos(adr, pos);
        
                pcl_pt.x = pos[0];
                pcl_pt.y = pos[1];
                pcl_pt.z = pos[2];
                pcl_pt.intensity = attention;
                local_sem_cloud->push_back(pcl_pt);
            }
    }

    std::list<Object> local_objects;    
    createObjects(local_sem_cloud, local_objects); // cluster the global point cloud 
    mergeObjects(global_objects, local_objects);

    std::chrono::duration<double> dt_obj = std::chrono::high_resolution_clock::now() - start;
    // ROS_INFO("Time obj: %f, local obj %d, global obj %d", dt_obj, local_objects.size(), global_objects.size());
}


void AttentionMap::loopTimer(const ros::TimerEvent& event){
    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();

    stem_msgs::Viewpoints vpts_msg;
    vpts_msg.viewpoints.header.stamp = ros::Time::now();
    vpts_msg.viewpoints.header.frame_id = "map";

    // Sample viewpoints around all objects
    std::list<std::vector<TargetViewpoint>> all_viewpoints;
    for (Object& object: global_objects){   
        if (!object.isInBox(local_box_min, local_box_max)) continue;
          
        std::vector<TargetViewpoint> sample_vpts; 
        sampleViewpoints(object, sample_vpts);
        all_viewpoints.push_back(sample_vpts);
    }
    
    // int i=0, j=0;
    // for (auto vpts: all_viewpoints){
    //     i++;
    //     std::vector<Eigen::Vector3d> vpt_positions;

    //     for (auto vpt: vpts){
    //         vpt_positions.push_back(vpt.posToEigen());
    //         j++;
    //     }
    //         viz.drawSpheres(vpt_positions, 0.1, Vector4d(0.5, 0.5, 1, 1), "points_"+std::to_string(i), i, 6);

    // }


    removeSimilarPosesFromList(all_viewpoints);
    
    // Calculate priority of each viewpoint 
    std::list<std::vector<TargetViewpoint>>::iterator iter = all_viewpoints.begin();
    for (Object& object: global_objects){      
        if (!object.isInBox(local_box_min, local_box_max)) continue;

        // calculate information gain
        findTopViewpoints(object, *iter);

        // Get first 3 three viewpoints for publishing
        for (int i=0; i<std::min((int)object.viewpoint_candidates.size(), 3); i++){
            vpts_msg.viewpoints.poses.push_back(object.viewpoint_candidates[i].toGeometryMsg());  
            vpts_msg.priorities.push_back(object.viewpoint_candidates[i].gain_);
        }
        iter++;
    }

    vpt_pub.publish(vpts_msg); // publish poses for use by lkh tsp inside fuel

    std::chrono::duration<double> dt_vpt = std::chrono::high_resolution_clock::now() - start;
    // ROS_INFO("Time vpts: %f", dt_vpt);

}

void sortViewpoints(std::vector<TargetViewpoint>& vpts){
    sort(vpts.begin(), vpts.end(), [](const TargetViewpoint& v1, const TargetViewpoint& v2) { return v1.gain_ > v2.gain_; });   
}


/*
Sample valid viewpoints around the object
A viewpoint is valid if:
1. exists in map
2. Not too close to unknown or occupied space
3. Has the object in view
*/
void AttentionMap::sampleViewpoints(Object& object, std::vector<TargetViewpoint>& sampled_vpts){


    // find the minimum radius for cylindrical sampling
        // find the smallest face of the bbox
        // find the AR of the smallest face
        // if this AR is larger than image ar
            // bound the face in image using the width of smallest face
        // else
            // bound using the height
        // find the rmin using the bound
    // object.viewpoint_candidates.clear();

    Eigen::Vector3d diag_3d = object.bbox_max_ - object.bbox_min_;
    int shortest_axis = diag_3d(1) > diag_3d(0) ? 0: 1;
    Eigen::Vector2d diag_2d = {diag_3d(shortest_axis), diag_3d(2)};
    
    // this distance might be obsolete now that we have isometric views. but still nice starting point
    double rmin = camera.getMinDistance(diag_2d); 

    rmin += diag_3d(1-shortest_axis)/2; // add the longer axis to rmin because the cylinder starts at the centroid
    
    for (double rc = rmin, dr = 0.4; rc <= rmin + 0.5 + 1e-3; rc += dr)
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

            TargetViewpoint vpt(TargetViewpoint(sample_pos, phi + M_PI, 0)); // 0 gain. will be computed later
            sampled_vpts.push_back(vpt);
        }
}


void AttentionMap::findTopViewpoints(Object& object, std::vector<TargetViewpoint>& sampled_vpts){
    // find the closest distance that would have the object still in view
    // sample viewpoints around the object starting from the minimum distance
    // for each sample 
        // if sample is near unknown region (OR) on an occupied cell (OR) not in the volume --> discard
        // if the object's perspective view from the sample doesnt fit in the camera --> discard 
        // evaluate information gain by ray casting (need attention buffer for this)
        // if gain is more than min gain, add viewpoint to candidate list
    // sort viewpoints by their gain
    // get some top viewpoints for the object
    object.viewpoint_candidates.clear();
    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();

    for (auto it=sampled_vpts.begin(); it!=sampled_vpts.end();++it){
            float gain = (int)computeInformationGain(object, it->pos_, it->yaw_);
            if (gain<=5) continue;
            
            it->gain_ = gain;
            object.viewpoint_candidates.push_back(*it);
        }
        
    if (!object.viewpoint_candidates.empty())
        sortViewpoints(object.viewpoint_candidates);       // sort list wrt information gain 
    else
        object.seen_=true;

    std::chrono::duration<double> dt_vpt = std::chrono::high_resolution_clock::now() - start;
    // ROS_INFO("Time vpt: %f", dt_vpt);

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
    Eigen::Vector3d bbox_min = object.bbox_min_ - Eigen::Vector3d(1,1,0.5);
    Eigen::Vector3d bbox_max = object.bbox_max_ + Eigen::Vector3d(1,1,0.5);

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

void AttentionMap::occCallback(const stem_msgs::uint8List& msg){
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

void AttentionMap::occInflateCallback(const stem_msgs::uint8List& msg){
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

bool AttentionMap::enforceIntensitySimilarity (const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
{
  // close enough + discrete enough + similar enough
  
  if (squared_distance > 1 || point_a.intensity<=att_min || point_b.intensity<=att_min || std::abs (point_a.intensity - point_b.intensity) > 1e-2f)
    return (false);
  return (true);
}


void AttentionMap::createObjects(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::list<Object>& objects){
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>(*global_att_cloud));
    
    // Apply clustering
    std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    // ec.setInputCloud(cloud_copy);
    // ec.setClusterTolerance(0.15); // Set the cluster tolerance (adjust as needed)
    // ec.setMinClusterSize(5);    // Set the minimum cluster size (adjust as needed)
    // ec.setMaxClusterSize(25000);   // (neSet the maximum cluster size (adjust as needed)
    // ec.extract(cluster_indices);


    pcl::ConditionalEuclideanClustering<pcl::PointXYZI> cec (true);
    cec.setInputCloud (cloud);
    // using namespace std::placeholders;

    cec.setConditionFunction (condfunc);
    cec.setClusterTolerance (0.15);
    cec.setMinClusterSize (5);
    cec.setMaxClusterSize (100);
    cec.segment (cluster_indices);
    // cec.getRemovedClusters (small_clusters, large_clusters);



    // ROS_INFO("num obj: %d", cluster_indices.size());
    int i = 0;
    for (const auto& cluster_index : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointIndices::Ptr pcl_indices(new pcl::PointIndices(cluster_index));
        extract.setInputCloud(cloud);
        extract.setIndices(pcl_indices);
        extract.filter(*cluster_cloud);
        
        // create objects from clustered clouds
        Object object(*cluster_cloud);
        object.id = ++i;
        objects.push_back(object);
    }
}

void AttentionMap::publishAttTimer(const ros::TimerEvent& e){
    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();

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

    std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dt = end-start;
    // ROS_INFO("Time for pub: %f", dt);

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


vector<Eigen::Vector3i> sixteenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(16);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  // z axis extend
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 2);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 2);
  
  // y axis
  neighbors[count++] = tmp - Eigen::Vector3i(0, 2, 0);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 2, 0);

    // x axis
  neighbors[count++] = tmp - Eigen::Vector3i(2, 0, 0);
  neighbors[count++] = tmp + Eigen::Vector3i(2, 0, 0);


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
    if (attention_buffer[sdf_map_->toAddress(nbr)]>att_min) return true;
  }
  return false;
}


bool AttentionMap::isFrontier(Eigen::Vector3i idx){
    // return knownfree(idx) && isNeighborUnknown(idx);
    return getOccupancy(idx)== fast_planner::SDFMap::UNKNOWN && isNeighborFree(idx);
}

// find all unique neihbhours around a cell that are frontiers
void AttentionMap::getNearbyFrontiers(Eigen::Vector3i idx, std::vector<Eigen::Vector3i>& frontier_nbrs){
    auto nbrs = sixteenNeighbors(idx); // 16 neighbors, increase this if frontiers aren't detected even though you see unknown space

    // std::vector<Eigen::Vector3i> frontier_nbrs;
    Eigen::Vector3d nbr_pos; 
    for (auto nbr : nbrs) {
        sdf_map_->indexToPos(nbr, nbr_pos);
        int nbr_adr = sdf_map_->toAddress(nbr);
        // ignore if not already checked or invalid frontier
        if (checked[nbr_adr] || nbr_pos(2)<0.1 || !sdf_map_->isInMap(nbr) || !isFrontier(nbr))
            continue;
        
        frontier_nbrs.push_back(nbr);
        checked[nbr_adr] = true;
    }
    // return frontier_nbrs;
}  

// Get diffused value from nearby voxels. Filters can be used
float AttentionMap::diffuse(Eigen::Vector3i bu_voxel){
    auto nbrs = allNeighbors(bu_voxel, 1); // 26 neighbors
    float att_nbr = 0.0; 
    float count = 0;
    Eigen::Vector3d nbr_pos;
    for (auto nbr : nbrs) {
        sdf_map_->indexToPos(nbr, nbr_pos);
        int nbr_adr = sdf_map_->toAddress(nbr);

        if (!sdf_map_->isInMap(nbr) || attention_buffer[nbr_adr] <att_min || nbr_pos(2)<0.1 )
            continue;
        
        att_nbr += attention_buffer[nbr_adr];
        count++;
    }
    float att_diffused = (count>0)? diffusion_factor*att_nbr/count:0;
    
    return att_diffused;
}

void gaussian_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered,
                    double radius){
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

        // pcl::VoxelGrid<pcl::PointXYZI> sor;
        // sor.setInputCloud (local_att_cloud_filtered);
        // sor.setLeafSize (0.1f, 0.1f, 0.1f);
        // sor.filter (*local_att_cloud_filtered);

    }
    
    
    // ======= Global map update ======

    // update the global buffer. Very small loop
    // float alpha = 0.2;
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
                float att_update = learning_rate*attention + (1-learning_rate)*attention_buffer[vox_adr];
                attention_buffer[vox_adr] = att_update; 
            }
        }
    }

}

// Diffuse attentive semantics into neighboring frontiers. 
// Run in a separate thread because computationally expensive and we dont need fast updates.
void AttentionMap::diffusionTimer(){

    while (true){
        time_point start = std::chrono::high_resolution_clock::now();
        time_diff dt;
        std::fill(checked.begin(), checked.end(), false); // TODO make this local 

        Eigen::Vector3i min_cut;
        Eigen::Vector3i max_cut;
        sdf_map_->posToIndex(local_box_min, min_cut);
        sdf_map_->posToIndex(local_box_max, max_cut);

        sdf_map_->boundIndex(min_cut);
        sdf_map_->boundIndex(max_cut);

        // print_eigen(local_box_max);

        Eigen::Vector3i idx;
        Eigen::Vector3d pos;

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y)
                for (int z = sdf_map_->mp_->box_min_(2); z < sdf_map_->mp_->box_max_(2); ++z) {

                int adr = sdf_map_->toAddress(x,y,z);
                float attention = attention_buffer[adr];
                
                if (attention <= att_min || occupancy_buffer_[adr] == fast_planner::SDFMap::FREE){ // this is the main thing that saves compute
                    attention_buffer[adr]=0;
                    continue;
                }
                sdf_map_->indexToPos(adr, pos);
                sdf_map_->posToIndex(pos, idx);

                std::vector<Eigen::Vector3i> bu_voxels;
                getNearbyFrontiers(Eigen::Vector3i(x, y, z), bu_voxels); // bottom up voxels: frontier cells that are near attentive regions
                // apply kernel to each unknown voxel

                for (auto& vox: bu_voxels){
                    int bu_voxel_adr = sdf_map_->toAddress(vox);

                    float att_diffused = diffuse(vox); // filter value
                    attention_buffer[bu_voxel_adr] = att_diffused>att_min?  (0.9*att_diffused + 0.1*attention_buffer[bu_voxel_adr]):0;     // the weighted update just allows it to stabilise a bit, otherwise a lot of flickering

                }
            }


        time_point end = std::chrono::high_resolution_clock::now();
        dt = end-start;
        // ROS_INFO("Time for diffusion: %f", dt);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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


void AttentionMap::odometryCallback(const nav_msgs::OdometryConstPtr&  msg){
    
    self_pos(0) = msg->pose.pose.position.x;
    self_pos(1) = msg->pose.pose.position.y;
    self_pos(2) = msg->pose.pose.position.z;

    // Eigen::Vector3d dir;
    // vel(0) = msg->twist.twist.linear.x;
    // vel(1) = msg->twist.twist.linear.y;
    // vel(2) = msg->twist.twist.linear.z;
    double yaw = quaternionMsgToYaw(msg->pose.pose.orientation);
    
    Eigen::Vector3d dir(cos(yaw), sin(yaw), 0);

    // update local box locations wrt world origin
    // offset in direction of travel
    local_box_min = self_pos - Eigen::Vector3d(2,2,0) + dir;
    local_box_max = self_pos + Eigen::Vector3d(2,2,0) + dir;
    
    local_box_min(2) = -0.2;
    local_box_max(2) = 1.5;

    // Uncomment to visualise local update box
    // viz.drawBox((local_box_min + local_box_max)/2.0, local_box_max - local_box_min, Eigen::Vector4d(0.5, 0, 1, 0.5), "box"+std::to_string(100), 100, 7);

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