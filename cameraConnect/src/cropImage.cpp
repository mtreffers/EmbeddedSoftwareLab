/**
Group number: 31
Student 1:
  Maurice Willemsen,4366662
Student 2:
  Michael Treffers, 4374614
*/

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

Mat cropImage(Mat image, int factor, int place){
  float height = image.size().height/factor;
  float startHeight = (factor-1-place)*height;

// Setup a rectangle to define your region of interest
  cv::Rect myROI(0,startHeight-1, image.size().width-1, height);

  // Crop the full image to that image contained by the rectangle myROI
  cv::Mat croppedImage = image(myROI);

  return croppedImage;
}
