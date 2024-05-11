#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

bool isPtInBox(const Eigen::Vector3d& point, const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max){
    for (int i=0; i<3; ++i){
        if (point(i) < bbox_min(i) || point(i) > bbox_max(i))
            return false;
    }
    return true;
}

double quaternionMsgToYaw(const geometry_msgs::Quaternion& quat) {
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}


geometry_msgs::Quaternion rpyToQuaternionMsg(const double& roll, const double& pitch, const double& yaw) {
    tf::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;

    quat_tf.setRPY(roll,pitch,yaw);
    quat_tf = quat_tf.normalize();
    // return tf::toMsg(quat_tf);

    tf::quaternionTFToMsg(quat_tf, quat_msg);

    return quat_msg;
}


#endif