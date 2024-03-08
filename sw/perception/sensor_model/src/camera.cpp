#include <sensor_model/camera.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <common/io.h>
Camera::Camera(const ros::NodeHandle& nh){
    // get these from the parameter server later on
    fx_ = 454;
    fy_ = 454;
    height_ = 480;
    width_ = 848;
    tol = 0.8;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // get all transforms here
    try{
        geometry_msgs::TransformStamped transform_msg = tfBuffer.lookupTransform("odom", "camera_link", ros::Time::now(), ros::Duration(2.0));
        T_odom_cam = tf2::transformToEigen(transform_msg);
    }
    catch (tf2::TransformException &ex){
        ROS_ERROR("Transform exception: %s", ex.what());
    }
    
}

Eigen::Vector2d Camera::project(const Eigen::Vector3d& point_cam){
    Eigen::Vector2d point_img;
    point_img(0) = point_cam(0)*fx_/point_cam(2); 
    point_img(1) = point_cam(1)*fy_/point_cam(2);

    return point_img;
}


/* Checks if point_3d is in the 2D view, and saves it in point_2d*/
bool Camera::isPtInView(const Eigen::Vector3d& point_cam){
    Eigen::Vector2d point_img =  project(point_cam);
    if ((abs(point_img(1)) > tol*height_/2) || (abs(point_img(0)) > tol*width_/2)){
        return false;
    }
    return true;
}

bool Camera::arePtsInView(const std::vector<Eigen::Vector3d>& points_cam){
    for (auto& point_cam: points_cam){
        if (!isPtInView(point_cam))
            return false;
    }
    return true;
}


/* in place transform from frame_from to frame_to */
void Camera::transform(const std::vector<Eigen::Vector3d>& points,std::vector<Eigen::Vector3d>& points_transformed, const Eigen::Isometry3d& t){
    for (uint i=0; i<points.size(); ++i){
        points_transformed[i] = t*points[i];
    }
}
