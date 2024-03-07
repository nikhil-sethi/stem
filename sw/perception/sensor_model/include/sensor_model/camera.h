#include <eigen3/Eigen/Eigen>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>


class Camera{
    public:
        Camera(const ros::NodeHandle& nh);
        bool isPtInView(const Eigen::Vector3d& point_3d, Eigen::Vector2d& point_2d);
        void transform(std::vector<Eigen::Vector3d>& points, const geometry_msgs::Transform& t);
        void transform(Eigen::Vector3d& point, const geometry_msgs::Transform& t);
        tf2::Transform T_odom_cam;

    private:
        double fx_;
        double fy_;
        double height_;
        double width_;
        float tol; // how much of the FOV can a point be at maximum

    
};

