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
Point2f thresh_callback(Mat image, int debug);
Mat cropImage(Mat image, int factor, int place);

/** @function main */
Point2f calc_line(Mat src, int measurePoints, int debug)
{

  Mat croppedImage = cropImage(src,5,0);

  imshow("crop",croppedImage);

  /// Convert image to gray and blur it
  cvtColor( croppedImage, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );
  threshold(src_gray,src_gray,0,255,THRESH_BINARY|THRESH_OTSU);

return thresh_callback(src_gray,debug);
}

/** @function thresh_callback */
Point2f thresh_callback(Mat image, int debug)
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  Scalar color;

  /// Detect edges using canny
  //Canny(image, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );

  /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false );
  }

  ///  Get the mass centers: (+1 is to prevent nan)
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/(mu[i].m00+1) , mu[i].m01/(mu[i].m00+1) );}

  /// Draw contours
  Mat drawing = Mat::zeros( image.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(125, 200), rng.uniform(125,255), rng.uniform(100,175) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }

//Calculate middle line
if(mc.size() == 1 && mu[0].m00 == image.rows*image.cols){
  return Point2f(0,0);
}

float mc_l = 0, mc_r = 0, mc_y = 0, linePos_x, linePos_y;
int mc_l_c = 0, mc_r_c = 0;

  for(int i = 0; i < mc.size();i++){
    float x = mc[i].x;
    float y = mc[i].y;

    if(x < image.cols/2){
      mc_r += x;
      mc_r_c++;
    }else{
      mc_l += x;
      mc_l_c++;
    }
    mc_y += y;
  }

  if(debug == 1){
    cout << mc_l << " " << mc_l_c << " " << mc_r << " " << mc_r_c << "\n";
  }

  if(mc_r_c == 0){
    linePos_x = 0;
  }else if(mc_l_c == 0){
    linePos_x = image.cols-1;
  }else{
    linePos_x = (2*(mc_l/mc_l_c)+2*(mc_r/mc_r_c)-image.cols)/2;
  }

  if(mc_l_c != 0 || mc_r_c != 0){
    linePos_y = mc_y/(mc_r_c + mc_l_c);
  }else{
    linePos_y = 0;
  }
  Point2f middle = Point2f(linePos_x,linePos_y);

     /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
     if(debug == 1){
  printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours.size(); i++ )
     {
       printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
       color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }

     circle(drawing,middle,4, color,-1,8,0);

     imshow("Contour",drawing);
   }

     return middle;
}
