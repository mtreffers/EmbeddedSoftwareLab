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

Mat cropImage(Mat image){
  // Setup a rectangle to define your region of interest
  cv::Rect myROI(0, (image.size().height/2)-1, image.size().width, image.size().height/2);

  // Crop the full image to that image contained by the rectangle myROI
  // Note that this doesn't copy the data
  cv::Mat croppedImage = image(myROI);

  namedWindow("CroppedImage",CV_WINDOW_AUTOSIZE);
  imshow("CroppedImage",croppedImage);

  return croppedImage;
}
