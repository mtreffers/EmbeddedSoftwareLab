#include <ros/ros.h>
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


// Function header
int** calc_line(Mat src, int measurePoints);

static std::string OPENCV_WINDOW = "Original image";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1,
      &ImageConverter::imageCb, this, image_transport::TransportHints("compressed"));
    image_pub_ = it_.advertise("/image_converter/opencv_image", 1);

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
    warpAffine(cv_ptr->image,image,getRotationMatrix2D(center,-90,1),cv_ptr->image.size());

  //original image
  namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
  imshow(OPENCV_WINDOW,image);

  //Detect lines
  int** out = calc_line(image,measurePoints);

      cv::waitKey(300);


    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
