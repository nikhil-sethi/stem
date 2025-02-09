#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <stem_msgs/target.h>
#include <vector>
#include <algorithm>
#include <string>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/msgs/geometry.pb.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <map>
// #include <Eigen/Eigen>
// #include <gazebo_plugins/pubQueue.h>

// AMRLAB ground truth
// std::map<std::string, float> priorities = {{"person", 5}, {"gun", 3}, {"bag", 3}, {"drugs", 4}};

//earthquake ground truth
// std::map<std::string, float> priorities = {
//     {"marker_1", 8}, // human
//     {"marker_10", 5}, // flashlight
//     {"marker_23", 4}, //plant
//     {"marker_3", 8}, //blood
//     {"marker_9", 5}, //rubble
//     {"marker_5", 5}, //table
//     {"marker_22", 5}, //carpet
//     {"marker_12", 8}, //dog
//     };

//mine ground truth
std::map<std::string, float> priorities = {
    {"marker_1", 8}, // human
    {"marker_10", 4}, // flashlight
    {"marker_19", 3}, //rope
    {"marker_3", 8}, //blood
    {"marker_11", 5}, //radio
    {"marker_12", 7}, //dog
    };


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
        ros::Publisher targets_gt_pub, markers_pub;
        ros::Timer timer;
        // PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;

        std::unique_ptr<ros::NodeHandle> nh;
        std::vector<stem_msgs::target> targets;
        visualization_msgs::MarkerArray marker_array_msg;
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
    this->targets_gt_pub = this->nh->advertise<stem_msgs::target>("/gazebo/targets/gt", 20);
    this->markers_pub = this->nh->advertise<visualization_msgs::MarkerArray>("/gazebo/gt/target_markers", 10);
    // create publish timer
    this->timer = this->nh->createTimer(ros::Duration(0.1), &TargetGTPlugin::targetPubTimer,this);        

    updateTargets(models);
    if (ros::isInitialized()){
        ros::spin();
    }
}

void TargetGTPlugin::updateTargets(physics::Model_V models){
    // Iterate over the models and print their names
    
    int i =0;
    for (const auto& model : models){
        std::string modelName = model->GetName();
        std::cout << "Model Name: " << modelName << std::endl;
        physics::Link_V links = model->GetLinks();

        for (auto& link: links){
            std::string linkName = link->GetName();

            // split class and instance (assuming _ as split)    
            // std::string label = splitClass(linkName, '_');
            std::string label = linkName.substr(0,linkName.size()-2);

            // check if target is relevant
            if (priorities.find(label) != priorities.end() && priorities[label] >0){
                
                // creat a target message
                stem_msgs::target target_msg;
                target_msg.label = linkName;
                ignition::math::AxisAlignedBox bbox = link->BoundingBox(); 
                ignition::math::Vector3d bbox_min = bbox.Min();
                ignition::math::Vector3d bbox_max = bbox.Max();
                target_msg.bbox_min = std::vector<double>({bbox_min[0], bbox_min[1], bbox_min[2]});
                target_msg.bbox_max = std::vector<double>({bbox_max[0], bbox_max[1], bbox_max[2]});
                target_msg.priority = priorities[label];
                std::cout << "link Name: " << linkName <<"priority: " << target_msg.priority << std::endl;
                
                // get the mesh filename
                msgs::Geometry geom = link->GetVisualMessage(modelName+"::"+linkName+"::visual").geometry();
                
                if (geom.has_mesh()){
                    std::string mesh_uri= geom.mesh().filename();             
                    // load into marker
                    visualization_msgs::Marker marker;
                    marker.ns = linkName;
                    marker.id = i;
                    marker.header.frame_id = "map";

                    ignition::math::Pose3d pose = link->WorldPose();
                    std::cout<<pose.X() <<" "<<pose.Y()<<std::endl;
                    marker.scale.x = 1.0;
                    marker.scale.y = 1.0;
                    marker.scale.z = 1.0;
                    marker.pose.position.x = pose.X();
                    marker.pose.position.y = pose.Y();
                    marker.pose.position.z = pose.Z();
                    marker.pose.orientation.x = pose.Rot().X();
                    marker.pose.orientation.y = pose.Rot().Y();
                    marker.pose.orientation.z = pose.Rot().Z();
                    marker.pose.orientation.w = pose.Rot().W();
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                    int idx = mesh_uri.find("://");
                    marker.mesh_resource = "file:///root/thesis_ws/src/thesis/sw/stem_bringup/models/"+ mesh_uri.substr(idx+3, 100);
                    marker.mesh_use_embedded_materials = true;
                    marker_array_msg.markers.push_back(marker);
                }
                i++;
                targets.push_back(target_msg);
                

                // physics::CollisionPtr collision = link->GetCollision();

                // // Get the mesh from the collision
                // const auto& mesh = collision->GetMesh();
                // if (!mesh)
                // {
                //     gzerr << "Model " << this->model->GetName() << " collision has no mesh." << std::endl;
                //     return;
                // }

                // // Get the vertices of the mesh
                // const auto& vertices = mesh->vertices;
                // for (const auto& vertex : vertices)
                // {
                //     gzmsg << "Vertex: (" << vertex.x << ", " << vertex.y << ", " << vertex.z << ")" << std::endl;
                // }


                // target_links.push_back()
            }
        }
    }
}

void TargetGTPlugin::targetPubTimer(const ros::TimerEvent& e){
    // stem_msgs::target target_msg;
    for (auto target_msg: targets){
        this->targets_gt_pub.publish(target_msg);
    }

    this->markers_pub.publish(marker_array_msg);

}

}
