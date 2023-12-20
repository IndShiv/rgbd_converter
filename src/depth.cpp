
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
  
}
void depthCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
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


    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat depth_mono8_img;
    depthToCV8UC1(depth_float_img, depth_mono8_img);
}
int main(int argc, char **argv)
{
    // Initiate a new ROS node named "listener"
	ros::init(argc, argv, "listener_node");
	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle node;


 
    image_transport::ImageTransport it(node);
    image_transport::Subscriber depth_sub = it.subscribe("camera/depth/image", 1, depthCallback);

    // Enter a loop, pumping callbacks
    ros::spin();

    return 0;
}
