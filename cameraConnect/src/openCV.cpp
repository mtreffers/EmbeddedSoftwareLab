#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

Mat src_gray; Mat tmp; Mat tmp_gray;
int thresh = 60;
int max_thresh = 255;
RNG rng(12345);

/// Function header
int* thresh_callback(Mat image);

/** @function main */
int** calc_line(Mat src, int measurePoints)
{
  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );
  threshold(src_gray,src_gray,0,255,THRESH_BINARY|THRESH_OTSU);

  int *tmp = thresh_callback(src_gray);

  return (int**)calloc(2,sizeof(int*));
}

/** @function thresh_callback */
int* thresh_callback(Mat image)
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  //Canny(image, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );

  /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false );
  }

  ///  Get the mass centers:
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/(mu[i].m00) , mu[i].m01/(mu[i].m00) );}

  /// Draw contours
  Mat drawing = Mat::zeros( image.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(125, 200), rng.uniform(125,255), rng.uniform(100,175) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }

     /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
  printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours.size(); i++ )
     {
       printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }

     imshow("Contour",drawing);

     return (int*)calloc(1,sizeof(int));
}
