#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <common_msgs/target.h>
#include <vector>
#include <algorithm>
#include <string>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector3.hh>
#include <map>
// #include <Eigen/Eigen>
// #include <gazebo_plugins/pubQueue.h>

std::map<std::string, float> priorities = {{"person", 5}, {"gun", 3}, {"bag", 3}, {"drugs", 4}};

// // simulate a cosine matrix/semantic graph
// int priority_function(std::string label){
//     if (label=="person") return 5;
//     else if (label=="drug") return 4;
//     else if (label=="gun") return 3;
//     else if (label=="bag") return 3;
//     else return 0;
// }

std::string splitClass(std::string linkName, char delim){
    std::string label = "";
    int i = 0;
    char c = linkName[i++]; 
    while (c!=delim){
        label += c;
        c = linkName[i++];
    }

    return label;
}

namespace gazebo
{
  class TargetGTPlugin : public WorldPlugin{
    public: 
        TargetGTPlugin();
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
        void targetPubTimer(const ros::TimerEvent& e);
        void updateTargets(physics::Model_V models);

    private:
        physics::Link_V links; // list of links in the model. Treated as models here because the world design
        ros::Publisher targets_gt_pub;
        ros::Timer timer;
        // PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;

        std::unique_ptr<ros::NodeHandle> nh;
        std::vector<common_msgs::target> targets;
  };
  GZ_REGISTER_WORLD_PLUGIN(TargetGTPlugin)

   
TargetGTPlugin::TargetGTPlugin(){
    // normalise priorities
    float sum = 0;
    // for (std::map<std::string, float>::const_iterator i= priorities.begin(); i!=priorities.end();i++){
    //     sum += i->second;
    // }
    
    // for (std::map<std::string, float>::iterator i= priorities.begin(); i!=priorities.end();i++){
    //     i->second /= sum;
    // }


}

void TargetGTPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
    // ros::init()
    // Get all models in the world
    physics::Model_V models = _world->Models();
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    nh.reset(new ros::NodeHandle("gazebo_client"));
    this->targets_gt_pub = this->nh->advertise<common_msgs::target>("/gazebo/targets/gt", 1);
    // create publish timer
    this->timer = this->nh->createTimer(ros::Duration(0.1), &TargetGTPlugin::targetPubTimer,this);        

    updateTargets(models);

}

void TargetGTPlugin::updateTargets(physics::Model_V models){
    // Iterate over the models and print their names
    for (const auto& model : models){
        std::string modelName = model->GetName();
        std::cout << "Model Name: " << modelName << std::endl;
        physics::Link_V links = model->GetLinks();

        for (auto& link: links){
            std::string linkName = link->GetName();

            // split class and instance (assuming _ as split)    
            std::string label = splitClass(linkName, '_');

            // check if target is relevant
            if (priorities.find(label) != priorities.end() && priorities[label] >0){
                
                // creat a target message
                common_msgs::target target_msg;
                target_msg.label = linkName;
                ignition::math::AxisAlignedBox bbox = link->BoundingBox(); 
                ignition::math::Vector3d bbox_min = bbox.Min();
                ignition::math::Vector3d bbox_max = bbox.Max();
                target_msg.bbox_min = std::vector<double>({bbox_min[0], bbox_min[1], bbox_min[2]});
                target_msg.bbox_max = std::vector<double>({bbox_max[0], bbox_max[1], bbox_max[2]});
                target_msg.priority = priorities[label];
                std::cout << "link Name: " << linkName <<"priority: " << target_msg.priority << std::endl;
                targets.push_back(target_msg);
                // target_links.push_back()
            }
        }
    }
}

void TargetGTPlugin::targetPubTimer(const ros::TimerEvent& e){
    // common_msgs::target target_msg;
    for (auto target_msg: targets){
        this->targets_gt_pub.publish(target_msg);
    }
    
}

}
