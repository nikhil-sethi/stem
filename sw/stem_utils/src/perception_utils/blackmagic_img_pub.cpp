#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <mutex>

// Global variables
const int WIDTH = 1920;
const int HEIGHT = 1080;

class ImagePublisherNode {
public:
    ImagePublisherNode() : nh_("~") {
        // Initialize ROS node handle and image transport
        image_transport::ImageTransport it(nh_);
        image_pub_ = it.advertise("camera/image_raw", 1);

        // Start ffmpeg process
        std::string ffmpeg_command = "ffmpeg -nostdin -i /dev/video2 -vf scale=" + std::to_string(WIDTH) + ":" + std::to_string(HEIGHT) +
                                     " -pix_fmt bgr24 -r 20 -an -vcodec rawvideo -f rawvideo -";

        ffmpeg_process_.reset(new std::thread([this, ffmpeg_command]() {
            FILE *pipe = popen(ffmpeg_command.c_str(), "r");
            if (!pipe) {
                ROS_ERROR("Failed to open ffmpeg subprocess.");
                return;
            }

            const int frame_size = WIDTH * HEIGHT * 3;
            unsigned char buffer[frame_size];

            while (ros::ok()) {
                size_t bytes_read = fread(buffer, 1, frame_size, pipe);
                if (bytes_read != frame_size) {
                    ROS_ERROR("Failed to read complete frame from ffmpeg.");
                    break;
                }

                cv::Mat frame(HEIGHT, WIDTH, CV_8UC3, buffer);

                std::lock_guard<std::mutex> lock(mutex_);
                frame_buffer_ = frame.clone();
                new_frame_available_ = true;
            }

            pclose(pipe);
        }));

        // Create a thread to publish images
        publish_thread_ = std::thread(&ImagePublisherNode::publishImages, this);
    }

    ~ImagePublisherNode() {
        // Shutdown ffmpeg process
        if (ffmpeg_process_->joinable()) {
            ffmpeg_process_->join();
        }
    }

    void publishImages() {
        ros::Rate rate(12);  // Publish rate

        while (ros::ok()) {
            if (new_frame_available_) {
                std::lock_guard<std::mutex> lock(mutex_);

                if (frame_buffer_.empty()) {
                    continue;
                }

                // Convert OpenCV image to ROS Image message
                cv_bridge::CvImage cv_image;
                cv_image.encoding = "bgr8";
                cv_image.image = frame_buffer_;
                cv_image.header.stamp = ros::Time::now();

                // Publish the image message
                image_pub_.publish(cv_image.toImageMsg());

                new_frame_available_ = false;
            }

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::Publisher image_pub_;
    std::unique_ptr<std::thread> ffmpeg_process_;
    std::thread publish_thread_;
    cv::Mat frame_buffer_;
    std::mutex mutex_;
    bool new_frame_available_ = false;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_publisher_node_cpp");
    ImagePublisherNode node;
    ros::spin();
    return 0;
}
