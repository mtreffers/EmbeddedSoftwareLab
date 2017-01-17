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

int line_detect(Mat src, std::string window);
int distinctColors(int number, Mat image);
Mat cropImage(Mat image);

static std::string OPENCV_WINDOW = "Open CV voor koosjeee";

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

Point center = Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2);
Mat image;
warpAffine(cv_ptr->image,image,getRotationMatrix2D(center,-90,1),cv_ptr->image.size());

    //Detect lines
    cropImage(image);
      distinctColors(10,image);
      line_detect(image, OPENCV_WINDOW);
      cv::waitKey(300);

    // Draw an example circle on the video stream
    /*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cvnamedWindow( source_window, CV_WINDOW_AUTOSIZE );namedWindow( source_window, CV_WINDOW_AUTOSIZE );namedWindow( source_window, CV_WINDOW_AUTOSIZE );_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/

    // Update GUI Window
    /*cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);*/

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
