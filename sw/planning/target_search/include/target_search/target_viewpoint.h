#include <eigen3/Eigen/Eigen>
#include <tf/transform_datatypes.h>
#include <active_perception/frontier_finder.h>
#include <common/utils.h>


struct TargetViewpoint: fast_planner::Viewpoint{
    float gain_;

    TargetViewpoint(Eigen::Vector3d pos, double yaw, float gain=0){
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
        
        // tf2::Quaternion quat_tf;
        // quat_tf.setRPY(0,0,yaw_);
        // quat_tf = quat_tf.normalize();
        // msg.orientation = tf2::toMsg(quat_tf);
    
        msg.orientation = rpyToQuaternionMsg(0, 0, yaw_);
        return msg;
    }   

    bool isClose(const TargetViewpoint& other){
        double dpsi = yaw_-other.yaw_;
        return (pos_-other.pos_).norm() < 0.4 && abs(std::min(dpsi, 2*M_PI-dpsi))<0.4;
    }

    Eigen::Vector4d getColor(float min, float max, Eigen::Matrix<double,4,4> colormap);

};



void removeSimilarPosesFromList(std::list<std::vector<TargetViewpoint>>& myList) {

     for (auto it = myList.begin(); it != myList.end(); ++it) {
        auto it2 = std::next(it);
        for (; it2 != myList.end(); ++it2) {
            for (auto it3 = it->begin(); it3!=it->end(); it3++) {
                for (auto it4 = it2->begin(); it4!=it2->end(); it4++) {
                    if (it3->isClose(*it4)) {
                        it2->erase(it4); // Remove the vector with similar pose
                        --it4; // Adjust the iterator after erasing
                        break; // No need to check other poses in vec2
                    }
                }
            }
        }
    }

}

Eigen::Vector4d TargetViewpoint::getColor(float min, float max, Eigen::Matrix<double, 4,4> colormap){
    float rel_gain  = std::max(0.0f, std::min((gain_-min)/(max-min), 1.0f)); // bounds prevent nan values
    int idx = (int)(rel_gain*2.999); // size of colormap is 3
    
    Eigen::Vector4d cmin = colormap.row(idx);
    Eigen::Vector4d cmax = colormap.row(idx+1);
    Eigen::Vector4d color = cmin + (rel_gain*3 - idx)*(cmax-cmin); 
    color(3) = 1;
    return color;
}