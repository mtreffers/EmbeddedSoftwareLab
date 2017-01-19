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
float std_speed = 1.5; //range:[-2,2]


// Function header
Point2f calc_line(Mat src, int measurePoints, int debug);
Point2f decide(Point2f middle, cv::Size size, float speed, int debug);

static std::string OPENCV_WINDOW = "Original image";

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Publisher chatter_pub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1,
      &ImageConverter::imageCb, this, image_transport::TransportHints("compressed"));
    chatter_pub = nh_.advertise<geometry_msgs::Twist>("prorobot", 100);

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
    // warpAffine(cv_ptr->image,image,getRotationMatrix2D(center,-90,1),cv_ptr->image.size());
    transpose(cv_ptr->image, image);
    flip(image, image,1);


  //original image
  namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
  imshow(OPENCV_WINDOW,image);

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
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

Point2f decide(Point2f middle, cv::Size size, float speed, int debug){
  float width = size.width;
  float x = middle.x;
  float dif;
  int sign;

  if(x < width/2){
    dif = width/2 - x;
    sign = -1;
  }else{
    dif = x - width/2;
    sign = 1;
  }

float interval = width/10;

  for(int i = 0; i < 5; i++){
    // cout << dif << " " << i*interval << " " << (i+1)*interval << " " << i << "\n";
    if((dif > i*interval) &&(dif < (i + 1)*interval)){
      Point2f out;// = (float*)calloc(2,sizeof(float));
      out.x = sign*(2.0/5.0)*i;
      out.y = (-(1.0/5.0)*i + 1.0)*speed;

if(debug == 1){
  cout << "Decide: " << out << "\n";
}

      return out;
    }
  }

}
