#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class AttentionMap2d{
    public:
        AttentionMap2d(ros::NodeHandle& nh){

            // Get params
            nh.param("/perception/attention_map/2d/low_H", low_H, 80);
            nh.param("/perception/attention_map/2d/low_S", low_S, 150);
            nh.param("/perception/attention_map/2d/low_V", low_V, 180);

            nh.param("/perception/attention_map/2d/high_H", high_H, 130);
            nh.param("/perception/attention_map/2d/high_S", high_S, 255);
            nh.param("/perception/attention_map/2d/high_V", high_V, 255);

            color_sub = nh.subscribe("/camera/color/image_raw/compressed", 10, &AttentionMap2d::attCallback, this);
            att_2d_pub = nh.advertise<sensor_msgs::CompressedImage>("/attention_map/2d/compressed", 10);
        }

        void attCallback(const sensor_msgs::CompressedImageConstPtr& msg);    

    private:
        cv_bridge::CvImagePtr cv_ptr;
        ros::Subscriber color_sub;
        ros::Publisher att_2d_pub;
        int low_H, low_S, low_V;
        int high_H, high_S, high_V;

};

void AttentionMap2d::attCallback(const sensor_msgs::CompressedImageConstPtr& msg){
    cv::Mat img_hsv, img_mask, blurred_image, morphed_image;

    // get image
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    // create segmentation mask
    
    cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);
    
    // cv::GaussianBlur(img_hsv, blurred_image, cv::Size(5, 5), 0, 0,  cv::BORDER_DEFAULT);

    // threshold within desired range
    cv::inRange(img_hsv, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), img_mask);

    // cv::GaussianBlur(img_mask, blurred_image, cv::Size(5, 5), 0);
    
    // close large holes with a big filter. this helps in removing the noise within the area of segmented objects.
    // I do this agressively here, and then follow by opening
    cv::morphologyEx(img_mask, morphed_image, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // open holes a little bit to account for actual large holes inside of objects.
    // this combined with closing above removes noise but retains large open areas. 
    cv::morphologyEx(morphed_image, morphed_image, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    // publish
    cv_ptr->image = morphed_image;
    att_2d_pub.publish(cv_ptr->toCompressedImageMsg());
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "att_2d_node");

    ros::NodeHandle nh;
    AttentionMap2d attention_map_2d(nh);
    
    ros::spin();
    return 0;
}
