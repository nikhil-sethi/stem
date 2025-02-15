#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
// #include <compressed_image_transport/compression_common.h>
#include <cv_bridge/cv_bridge.h>


class DepthImageCompressor
{
public:
    DepthImageCompressor(ros::NodeHandle& nh)
    : it_(nh)
    {
        depth_image_sub_ = it_.subscribe("camera/depth/image_rect_raw", 1, &DepthImageCompressor::depthImageCallback, this);
        compressed_image_pub_ = it_.advertise("camera/depth/image_raw", 1);
    }

private:
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // sensor_msgs::CompressedImage compressed_msg;
        // compressed_msg.header = msg->header;
        // compressed_msg.format = compressed_depth_image_transport::format::DEPTH;

        // try
        // {
        //     // Use compressed_depth_image_transport to compress the depth image
        //     // compressed_depth_image_transport::CompressionConfig compression_config;
        //     // compression_config.format = compressed_depth_image_transport::format::PNG;
		// 	// compression_config.png_level = 5;
        //     // compressed_depth_image_transport::encodeCompressedDepthImage(*msg, compressed_msg, compression_config);
		// 	compressed_image_pub_.publish(msg);
		// }
        // catch (const std::exception& e)
        // {
        //     ROS_ERROR("Failed to compress depth image: %s", e.what());
        //     return;
        // }
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);

        if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
          (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, 1000);

        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
        cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Or whatever

        compressed_image_pub_.publish(cv_ptr->toImageMsg());
        // compressed_image_pub_.publish(msg);
    }

    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_image_sub_;
    image_transport::Publisher compressed_image_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_image_compressor");
    ros::NodeHandle nh;
    DepthImageCompressor dic(nh);
    ros::spin();
    return 0;
}
