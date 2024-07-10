// #include <opencv/core.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>


cv::Mat* depth_img = new cv::Mat(480,848, CV_16UC1);
// depth_img->setTo(cv::Scalar::all(1));
// resolution is set to 848 x 480
int _width = 848;
int _height = 480;
bool is_disparity = false;
float min_depth = 0.29f; // to avoid depth inversion, it’s offset from 0.3 to 0.29. Please see Figure 7
float max_depth = 5.0f;
float min_disparity = 1.0f / max_depth;
float max_disparity = 1.0f / min_depth;
unsigned short hue_value = 0; // from 0 to 255 * 6 - 1 = 0-1529 by Hue colorization


unsigned int RGBtoD(unsigned int r, unsigned int g, unsigned int b)
{   
	// conversion from RGB color to quantized depth value
	if (b + g + r < 255)
	{
		return 0;
	}
	else if (r >= g && r >= b)
	{
		if (g >= b)
		{	
			return g - b;
		}
		else
		{
			return (g - b) + 1529;
		}
	}
	else if (g >= r && g >= b)
	{
		return b - r + 510;
	}
	else if (b >= g && b >= r){
		std::cout<<"dfg"<<r - g + 1020<<std::endl;
        return r - g + 1020;
        
	}
}


void img_callback(const sensor_msgs::CompressedImageConstPtr& img){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img);
	cv_ptr->image.copyTo(*depth_img);
    // cv_ptr->image.a(*depth_image_);
    // Convert the compressed image to a cv::Mat
    // cv::Mat depth_img = cv::imdecode(cv::Mat(img->data), cv::IMREAD_ANYCOLOR);
            
    // cv_ptr->image.copyTo(*depth_img);
    
    // row = depth_img->ptr<uint16_t>(240);
    uint16_t px = depth_img->at<uint16_t>(240, 480);

    // unsigned int R = px[2];
    // unsigned int G = px[1];
    // unsigned int B = px[0];

    // unsigned int hue_value = RGBtoD(R, G, B);
    // unsigned int z_value = static_cast<unsigned int>((min_depth + (max_depth - min_depth) * hue_value / 1529.0f) + 0.5f);

    // std::cout<<R<<" "<<G<<" "<<B<<std::endl;
    std::cout<<px<<std::endl;

}



// unsigned char* in = reinterpret_cast<const unsigned char*>input_color_data_array;
// unsigned short* out = reinterpret_cast<unsigned short*>output_depth_data_array;

// for (int i = 0; i < _height; i++)
// {
// 	for (int j = 0; j < _width; j++)
// 	{
// 		unsigned char R = *in++;
// 		unsigned char G = *in++;
// 		unsigned char B = *in++;

// 		unsigned short hue_value = RGBtoD(R, G, B);

// 		if(out_value > 0)
// 		{
// 			if(!is_disparity)
// 			{
// 				unsigned short z_value = static_cast<unsigned short>((min_depth + (max_depth - min_depth) * hue_value / 1529.0f) + 0.5f);
// 				out++ = z_value;
// 			}
// 			else
// 			{
// 				float disp_value = min_disparity + (max_disparity - min_disparity) * out_value / 1529.0f;
// 				*out++ = static_cast<unsigned short>((1.0f / disp_value) / depth_units + 0.5f);
// 			}
// 		}
// 		else
// 		{
// 			*out++ = 0;
// 		}
// 	}
// }


int main(int argc, char** argv){
    ros::init(argc, argv, "depth_compress_node");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::CompressedImage>("/camera/depth/image_rect_raw/compressed", 10, img_callback);

    ros::spin();   
    return 0;
}