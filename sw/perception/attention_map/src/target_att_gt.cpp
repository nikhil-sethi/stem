#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <common_msgs/target.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <plan_env/sdf_map.h>
#include <active_perception/frontier_finder.h>

std::vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel) {
  std::vector<Eigen::Vector3i> neighbors(10);
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



class AttGTMaker{
    public:
        fast_planner::SDFMap sdf_map_;
        std::vector<common_msgs::target> targets;
        std::vector<float> attention_buffer;
        std::vector<bool> checked;
        ros::Publisher att_gt_pub;
        ros::Subscriber target_sub;
        ros::Timer att_pub_timer, att_update_timer;
        double diffusion_factor, att_min;

        AttGTMaker(ros::NodeHandle&  nh){
            // sdf_map_ = fast_planner::SDFMap();
            sdf_map_.setParams(nh);
            attention_buffer = std::vector<float>(sdf_map_.buffer_size, 0);
            checked = std::vector<bool>(sdf_map_.buffer_size, false);

            // let the targets come in
            ros::topic::waitForMessage<common_msgs::target>("/gazebo/targets/gt");
            target_sub = nh.subscribe<common_msgs::target>("/gazebo/targets/gt", 20, &AttGTMaker::targetCallback, this);

            // diffusion timer
            att_update_timer = nh.createTimer(ros::Duration(0.1), &AttGTMaker::attUpdateTimer, this);

            // start the ground truth map publish
            att_gt_pub = nh.advertise<sensor_msgs::PointCloud2>("/attention_map/gt", 10);
            att_pub_timer = nh.createTimer(ros::Duration(0.1), &AttGTMaker::attPubTimer, this);
            
            diffusion_factor = 0.9;
            att_min = 1;
        }

        void targetCallback(const common_msgs::target msg){
            Eigen::Vector3i bbox_min, bbox_max;
            Eigen::Vector3d bbox_mind(msg.bbox_min[0], msg.bbox_min[1],msg.bbox_min[2]); 
            Eigen::Vector3d bbox_maxd(msg.bbox_max[0], msg.bbox_max[1],msg.bbox_max[2]); 
            sdf_map_.posToIndex(bbox_mind, bbox_min);
            sdf_map_.posToIndex(bbox_maxd, bbox_max);
            // ROS_INFO("")
            // std::cout<<"sfg"<<std::endl;
            Eigen::Vector3i pos;
            for (int i = bbox_min(0); i<bbox_max(0); i++)
                for (int j = bbox_min(1); j<bbox_max(1); j++)
                    for (int k = bbox_min(2); k<bbox_max(2); k++){
                        if (!sdf_map_.isInMap(Eigen::Vector3i(i, j, k)))
                            continue;
                        int adr = sdf_map_.toAddress(i, j, k);
                        attention_buffer[adr] = msg.priority;
                }     


        }
        bool isNeighborAttentive(const Eigen::Vector3i& voxel) {
            // At least one neighbor is unknown
            auto nbrs = sixNeighbors(voxel);
            for (auto nbr : nbrs) {
                if (attention_buffer[sdf_map_.toAddress(nbr)]>att_min) return true;
            }
            return false;
        }

        // diffuse value at pos into neighbors
        void diffuse(int adr){
            Eigen::Vector3i idx;
            Eigen::Vector3d pos;
            sdf_map_.indexToPos(adr, pos);
            sdf_map_.posToIndex(pos, idx);

            auto nbrs = allNeighbors(idx, 1); // 10 neighbors
            float att_nbr = 0.0; 
            float count = 0;
            Eigen::Vector3d nbr_pos; // need a new pos var so the att map doesn't get overriden
            for (auto nbr : nbrs) {
                sdf_map_.indexToPos(nbr, nbr_pos);
                if (!sdf_map_.isInMap(nbr))
                    continue;
                    
                // calculate nearby attention
                int nbr_adr = sdf_map_.toAddress(nbr);
                // att_nbr += attention_buffer[nbr_adr] >0 ? attention_buffer[nbr_adr]: 0;
                
                // save list of unknown voxels near attentive regions
                // if (!checked[nbr_adr]){
                //     // bu_voxels.push_back(nbr_adr);

                //     auto nbrs2 = allNeighbors(idx, 1); // 26 neighbors
                //     float att_nbr2 = 0.0; 
                //     float count2 = 0;
                //     for (auto nbr2 : nbrs2) {
                sdf_map_.indexToPos(nbr, nbr_pos);
                //         int nbr2_adr = sdf_map_.toAddress(nbr2);
                if (attention_buffer[nbr_adr] < att_min || !sdf_map_.isInMap(nbr) )
                    continue;
                        
                att_nbr += diffusion_factor*attention_buffer[nbr_adr];
                count++;
                //     }
                
                    
                //     // the weighted update just allows it to stabilise a bit, otherwise a lot of flickering
                // checked[nbr_adr] = true;
                // }
            
            }
            float att_diffused = (count>0)? att_nbr/count:0;
            attention_buffer[adr] = att_diffused>att_min?  (0.9*att_diffused + 0.1*attention_buffer[adr]):0;

        }

        void attUpdateTimer(const ros::TimerEvent& e){
            std::fill(checked.begin(), checked.end(), false);

            // diffusion
            for (int i=0; i<attention_buffer.size(); i++){
                Eigen::Vector3d pos;
                Eigen::Vector3i idx;
                sdf_map_.indexToPos(i, pos);
                sdf_map_.posToIndex(pos, idx);
                
                if (attention_buffer[i]<att_min && isNeighborAttentive(idx))
                    diffuse(i);
            }
        }

        void attPubTimer(const ros::TimerEvent& e){
            pcl::PointXYZI pt;
            Eigen::Vector3d pos;
            pcl::PointCloud<pcl::PointXYZI> att_cloud;
            for (int i = 0; i < attention_buffer.size(); ++i) {
                if (attention_buffer[i]<=att_min)
                    continue;
                sdf_map_.indexToPos(i, pos);
                pt.x = pos[0];
                pt.y = pos[1];
                pt.z = pos[2];
                pt.intensity = attention_buffer[i];
                att_cloud.push_back(pt);
                }
            att_cloud.width = att_cloud.points.size();
            att_cloud.height = 1;
            att_cloud.is_dense = true;
            att_cloud.header.frame_id = "world";
            sensor_msgs::PointCloud2 att_cloud_msg;
            pcl::toROSMsg(att_cloud, att_cloud_msg);
            att_gt_pub.publish(att_cloud_msg);
            // ROS_INFO("sdag");
        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "target_gt_node");
    ros::NodeHandle nh("/");
    AttGTMaker node(nh);
    ros::spin();


}