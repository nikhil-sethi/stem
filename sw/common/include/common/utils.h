#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

inline bool isPtInBox(const Eigen::Vector3d& point, const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max){
    for (int i=0; i<3; ++i){
        if (point(i) < bbox_min(i) || point(i) > bbox_max(i))
            return false;
    }
    return true;
}

inline double quaternionMsgToYaw(const geometry_msgs::Quaternion& quat) {
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}


inline geometry_msgs::Quaternion rpyToQuaternionMsg(const double& roll, const double& pitch, const double& yaw) {
    tf::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;

    quat_tf.setRPY(roll,pitch,yaw);
    quat_tf = quat_tf.normalize();
    // return tf::toMsg(quat_tf);

    tf::quaternionTFToMsg(quat_tf, quat_msg);

    return quat_msg;
}

// Interpolate a scalar to a color given a color map
inline Eigen::Vector4d getColor(float value, float min, float max, Eigen::Matrix<double, 4,4> colormap){
    float rel_gain  = std::max(0.0f, std::min((value-min)/(max-min), 1.0f)); // bounds prevent nan values
    int idx = (int)(rel_gain*(colormap.rows()-0.001)); // size of colormap 
    
    Eigen::Vector4d cmin = colormap.row(idx);
    Eigen::Vector4d cmax = colormap.row(idx+1);
    Eigen::Vector4d color = cmin + (rel_gain*3 - idx)*(cmax-cmin); 
    color(3) = 1;
    return color;
}

#endif