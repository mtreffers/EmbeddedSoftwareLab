/**
Group number: 31
Student 1:
  Maurice Willemsen, 4366662
Student 2:
  Michael Treffers, 4374614
*/

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

/// Function header
Point2f calc_middle(Mat image, int debug);
Mat cropImage(Mat image, int factor, int place);

// This function converts image and passes its through to the next function that calculates the middle point of a line
Point2f calc_line(Mat src, int measurePoints, int debug)
{
  Mat src_gray; Mat tmp; Mat tmp_gray;

  Mat croppedImage = cropImage(src,5,0);

if(debug == 1){
  imshow("crop",croppedImage);
}

  /// Convert image to gray, blur it and convert to black/white using threshold
  cvtColor( croppedImage, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );
  threshold(src_gray,src_gray,0,255,THRESH_BINARY|THRESH_OTSU);

return calc_middle(src_gray,debug);
}

// This function calculates the coordinate of the middle of the line
Point2f calc_middle(Mat image, int debug)
{
  RNG rng(12345);
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  Scalar color;

  /// Find contours
  findContours( image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );

  /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false );
  }

  ///  Get the mass centers: (+1 is to prevent NaN)
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

float mc_l = 0, mc_r = 0, mc_y = 0, linePos_x, linePos_y;
int mc_l_c = 0, mc_r_c = 0;

// For all mass centers check if they are in the left or right plane. For all centers in those planes calculate the average mass center.
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

// If there is no center on the left make the position all the way to the right to not lose the line and vice versa.
// If there are centers on both sides the mean of the inner bounderies of the average contour in both planes is taken.
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

// Show some debug information and show the images
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

     if(debug ==1){
     imshow("Contour",drawing);
   }
   }

   //return the midfle point
     return middle;
}

// This function decides what to do with the middle point
Point2f decide(Point2f middle, cv::Size size, float speed, int debug){
  float width = size.width;
  float x = middle.x;
  float dif;
  int sign;

// First it decides if the point is on the left or the right
  if(x < width/2){
    if(debug == 1){
      cout << "left\n ";
    }
    dif = width/2 - x;
    sign = 1;
  }else{
    if(debug == 1){
      cout << "right\n ";
    }
    dif = x - width/2;
    sign = -1;
  }

// Then the amount of steering and speed(depends on standard speed and steering)

// Devide the left/right plane in 10 parts and check in which part the line is
  float interval = width/10;

  for(int i = 0; i < 5; i++){
    if((dif >= i*interval) &&(dif <= (i + 1)*interval)){
      Point2f out;
      out.y = sign*(1.0/5.0)*i;
      out.x = (-(1.0/5.0)*i + 1.0)*speed;

      if(debug == 1){
        cout << "Decide: " << out << " i=" << i << "\n";
      }

// Return the amount of steering and speed as a point
      return out;
    }
  }

}
