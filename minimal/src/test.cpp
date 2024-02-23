#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  std::unique_ptr<cv::Mat> img;

public:
  ImageConverter()
    : it_(nh_)
  {
    img.reset(new cv::Mat);
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/attention_map/2d", 1,
      &ImageConverter::imageCb, this);
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    cv_ptr->image.copyTo(*img);
    double minVal; 
    double maxVal; 
    cv::Point minLoc; 
    cv::Point maxLoc;
    // ROS_ERROR("%d", img->depth());
    // if (img->depth()==CV_8S){
    //     ROS_ERROR("sdgs");
    // }
    // cv::minMaxLoc(*img, &minVal, &maxVal, &minLoc, &maxLoc);

    // uint8_t pix = *(img->ptr<uint8_t>(128)+207);

    float max=0;
    int um,vm;
    uint8_t* row;
    for (int u = 0; u<img->rows; ++u){
        row = img->ptr<uint8_t>(u); 
        for (int v = 0; v<img->cols; ++v){ 
            // uint8_t val = img->at<uint8_t>(u, v);
            // row += 1;
            float val = row[v]/255.0f;
            // ROS_ERROR("pix: %d, u %d, v %d", val, u, v);
            if (val>max){
                max = val;
                vm = v;
                um = u;
            }
            // ROS_ERROR("pix: %d, u %d, v %d", val, j, i);
        }
    }
    // uint8_t pix = img->at<uint8_t>(207, 384); 
    uint8_t pix = *(img->ptr<uint8_t>(207)+ 384); 
    ROS_ERROR("pix: %d", pix);
    ROS_ERROR("max u %d", um);
    ROS_ERROR("max v %d", vm);
    // ROS_ERROR("max %d", max);
    ROS_ERROR("max %f", max);


    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}