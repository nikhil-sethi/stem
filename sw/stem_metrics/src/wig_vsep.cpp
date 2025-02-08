/*

init mapros with params, topic name
load gt diffusion map

*/

#include "plan_env/map_ros.h"
#include "plan_env/sdf_map.h"
int main(int argc, char** argv){
    ros::init(argc, argv, "wig_vsep_node");
    ros::NodeHandle nh("/");

    fast_planner::SDFMap* sdf_map = new fast_planner::SDFMap();
    // sdf_map->setParams(nh);
    
    fast_planner::MapROS* mr = new fast_planner::MapROS();
    sdf_map->initMap(mr, nh); // also sets map for MR internally
    std::cout<<"buffersize:"<< sdf_map->buffer_size<<std::endl;
    mr->init(nh);

    ros::spin();



}