#include <active_perception/diffuser.h>
#include <plan_env/priority_map.h>
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

class DiffusionMapGT{
    public:
    std::shared_ptr<PriorityMap> priority_map;
    std::shared_ptr<Diffuser> diffuser;
    std::shared_ptr<fast_planner::SDFMap> sdf_map_;

    friend Diffuser;
    void createGTPriorityMap(const common_msgs::target msg);
    void doDiffusion();
    void pubTimer(const ros::TimerEvent& e);
    ros::Publisher att_gt_pub;
    ros::Subscriber target_sub;
    ros::Timer att_pub_timer;

    DiffusionMapGT(ros::NodeHandle&  nh){
        sdf_map_.reset(new fast_planner::SDFMap);
        sdf_map_->setParams(nh);

        std::cout<<"fg"<<std::endl;
        priority_map.reset(new PriorityMap);
        priority_map->setSDFMap(sdf_map_);
        priority_map->init(nh);
        std::cout<<"fgdf"<<std::endl;
        diffuser.reset(new Diffuser(nh));
        diffuser->setPriorityMap(priority_map);
        
        diffuser->setSDFMap(sdf_map_);

        auto targets_msg = ros::topic::waitForMessage<common_msgs::target>("/gazebo/targets/gt");
        // createGTPriorityMap(*targets_msg);
        target_sub = nh.subscribe<common_msgs::target>("/gazebo/targets/gt", 20, &DiffusionMapGT::createGTPriorityMap, this);

        // doDiffusion();
        std::cout<<"ffg"<<std::endl;
        att_gt_pub = nh.advertise<sensor_msgs::PointCloud2>("/attention_map/gt", 10);
        att_pub_timer = nh.createTimer(ros::Duration(0.1), &DiffusionMapGT::pubTimer, this);
    }
};

    void DiffusionMapGT::createGTPriorityMap(const common_msgs::target msg){
        Eigen::Vector3i bbox_min, bbox_max;
        Eigen::Vector3d bbox_mind(msg.bbox_min[0], msg.bbox_min[1],msg.bbox_min[2]); 
        Eigen::Vector3d bbox_maxd(msg.bbox_max[0], msg.bbox_max[1],msg.bbox_max[2]); 
        sdf_map_->posToIndex(bbox_mind, bbox_min);
        sdf_map_->posToIndex(bbox_maxd, bbox_max);
        // ROS_INFO("")
        // std::cout<<"sfg"<<std::endl;
        Eigen::Vector3i pos;
        for (int i = bbox_min(0); i<bbox_max(0); i++)
            for (int j = bbox_min(1); j<bbox_max(1); j++)
                for (int k = bbox_min(2); k<bbox_max(2); k++){
                    if (!sdf_map_->isInMap(Eigen::Vector3i(i, j, k)))
                        continue;
                    int adr = sdf_map_->toAddress(i, j, k);
                    priority_map->priority_buffer[adr] = msg.priority;
        }


        // doDiffusion();        
    }

    void DiffusionMapGT::doDiffusion(){
        for (uint i=0; i<diffuser->diffusion_buffer.size(); i++){
            Eigen::Vector3i idx;
            Eigen::Vector3d pos;
            sdf_map_->indexToPos(i, pos);
            sdf_map_->posToIndex(pos, idx);

            if (pos(2)<=0.2) continue;

            float conv = diffuser->partialConvolution(idx);
            // if (conv>8)
                // std::cout<<conv<<std::endl;
            diffuser->diffusion_buffer[i]  =  0.9f*conv + 0.1f*diffuser->diffusion_buffer[i];
        }
    }

    void DiffusionMapGT::pubTimer(const ros::TimerEvent& e){
        doDiffusion();

        pcl::PointXYZI pt;
        Eigen::Vector3d pos;
        pcl::PointCloud<pcl::PointXYZI> att_cloud;
        for (int i = 0; i < diffuser->diffusion_buffer.size(); ++i) {
            // if (attention_buffer[i]<=att_min)
            //     continue;
            sdf_map_->indexToPos(i, pos);
            pt.x = pos[0];
            pt.y = pos[1];
            pt.z = pos[2];
            pt.intensity = diffuser->diffusion_buffer[i];
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



int main(int argc, char **argv){
    ros::init(argc, argv, "diff_gt_node");
    ros::NodeHandle nh;
    DiffusionMapGT diff_mapt_gt(nh);
    ros::spin();

    return 0;
}  