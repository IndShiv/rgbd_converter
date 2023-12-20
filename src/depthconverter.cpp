#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



class DepthConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  DepthConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1, &DepthConverter::depthCallback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
  }



  // Function that converts the image to the appropriate image type 
  void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img)
  {
  // Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols)
    {
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);
    }
    cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mono8_img).toImageMsg();
    // publish this image
    image_pub_.publish(msg);
    
    
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& original_image)
  {
    cv_bridge::CvImagePtr cv_ptr;
    // Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    //Copy the image.data to imageBuf.
    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat depth_mono8_img;
    depthToCV8UC1(depth_float_img, depth_mono8_img);
    //image_pub_.publish(cv_ptr->toImageMsg());

  }



  
  
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_converter");
  DepthConverter dc;
  ros::spin();
  return 0;
}