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
Mat thresh_callback(Mat image, std::string window);

/** @function main */
int line_detect(Mat src, std::string window)
{

  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  /// Create Window
  char source_window[7] = "Source";
  //namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  //imshow( window, src );

  //createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh, thresh_callback );


  tmp = thresh_callback(src_gray, window);
  cvtColor( tmp, tmp_gray, CV_BGR2GRAY );
  blur( tmp_gray, tmp_gray, tmp_gray.size() );
  Mat tmp2 = thresh_callback(tmp_gray,"window2");

  Mat out;
  addWeighted(tmp,0.5,src,0.5,0, out,0);
  imshow(window,out);

  /*Mat out2;
  addWeighted(tmp2,0.5,src,0.5,0, out2,0);
  namedWindow(window+"2",CV_WINDOW_AUTOSIZE);
  imshow(window+"2",out2);*/

  return(0);
}

/** @function thresh_callback */
Mat thresh_callback(Mat image, std::string window)
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny(image, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }

  ///  Get the mass centers:
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }

  /// Show in a window
  //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  //imshow( "Contours", drawing );
  /*Mat out;
  addWeighted(drawing,0.5,image,0.5,0, out,0);
imshow(window,image);*/
  //imshow(window,out);


  /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
  /*printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours.size(); i++ )
     {
       printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }*/

     return drawing;
}
