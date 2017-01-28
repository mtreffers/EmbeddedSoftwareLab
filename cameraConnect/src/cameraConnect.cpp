/**
Group number: 31
Student 1:
  Maurice Willemsen,4366662
Student 2:
  Michael Treffers, 4374614
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

//Global variables
int measurePoints =3;
int debug = 1;
float std_speed = 1.1; //range:[-2,2]
static std::string OPENCV_WINDOW = "Original image";


// Function header
Point2f calc_line(Mat src, int measurePoints, int debug);
Point2f decide(Point2f middle, cv::Size size, float speed, int debug);

class ImageConverter
{
  // Create publisher and Subscriber
  ros::NodeHandle nh_;
  ros::Publisher chatter_pub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
  : it_(nh_)
  {
    // Subscribe to input and make publisher to advertise twist messages on /cmd-vel
    image_sub_ = it_.subscribe("/camera/image", 1,
    &ImageConverter::imageCb, this, image_transport::TransportHints("compressed"));
    chatter_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    cv::namedWindow(OPENCV_WINDOW);
  }

  // Destroy window on closing program
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //Try to read and store ros message
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //Rotate image
    Point center = Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2);
    Mat image;
    transpose(cv_ptr->image, image);
    flip(image, image,1);

    //Show original image
    if(debug ==1){
      namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
      imshow(OPENCV_WINDOW,image);
    }

    //Detect lines
    Point2f middle = calc_line(image,measurePoints,debug);

    //Decide what to do with image
    Point2f movement = decide(middle,image.size(),std_speed, debug);

    if(debug == 1){
      cv::waitKey(300);
    }

    //Construct twist message
    geometry_msgs::Twist message;
    message.linear.x = movement.x;
    message.angular.z = movement.y;

    //Send twist message to the robot
    chatter_pub.publish(message);
  }
};

int main(int argc, char** argv)
{
  // Initialize node and send/receive ROS messages
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
