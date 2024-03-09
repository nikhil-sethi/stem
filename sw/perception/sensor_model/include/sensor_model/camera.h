#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>


class Camera{
    public:
        Camera(const ros::NodeHandle& nh);
        bool isPtInView(const Eigen::Vector3d& point_3d);
        void transform(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& points_transformed, const Eigen::Isometry3d& t);
        bool arePtsInView(const std::vector<Eigen::Vector3d>& points_cam);
        Eigen::Vector2d project(const Eigen::Vector3d& point_cam);
        double getMinDistance(Eigen::Vector2d diag_2d);

        Eigen::Isometry3d T_odom_cam;

    private:
        double fx_;
        double fy_;
        double height_;
        double width_;
        float tol; // how much of the FOV can a point be at maximum
        double AR_;
    
};

