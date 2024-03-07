#include <sensor_model/camera.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
        tf2::fromMsg(transform_msg.transform, T_odom_cam);
    }
    catch (tf2::TransformException &ex){
        ROS_ERROR("Transform exception: %s", ex.what());
    }
    
}

/* Checks if point_3d is in the 2D view, and saves it in point_2d*/
bool Camera::isPtInView(const Eigen::Vector3d& point_cam, Eigen::Vector2d& point_img){
    point_img(0) = point_cam(0)*fx_/point_cam(2); 
    point_img(1) = point_cam(1)*fy_/point_cam(2);

    if ((abs(point_img(1)) > tol*height_) || (abs(point_img(0)) > tol*width_)){
        return false;
    }
    return true;
}


/* in place transform from frame_from to frame_to */
void Camera::transform(std::vector<Eigen::Vector3d>& points, const geometry_msgs::Transform& t){
    // Eigen::MatrixXd points_frame_to;
    try{
        for (uint i=0; i<points.size(); ++i){
            transform(points[i], t);
        }
    }
    catch (tf2::TransformException &ex){
        ROS_ERROR("Transform exception: %s", ex.what());
    }

}

// transform a point to a frame represented by 
void Camera::transform(Eigen::Vector3d& point, const geometry_msgs::Transform& t){
    geometry_msgs::TransformStamped tf_stamped_msg;
    tf_stamped_msg.child_frame_id = "camera_link";
    tf_stamped_msg.header.stamp = ros::Time::now();
    tf2::doTransform(point, point, tf_stamped_msg); 
}