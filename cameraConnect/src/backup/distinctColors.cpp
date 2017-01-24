#include <iostream>
#include <vector>
#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

void distinctColors(Mat image){
  int div = 75;
      int nl = image.rows;                    // number of lines
      int nc = image.cols * image.channels(); // number of elements per line

      for (int j = 0; j < nl; j++)
      {
          // get the address of row j
          uchar* data = image.ptr<uchar>(j);

          for (int i = 0; i < nc; i++)
          {
              // process each pixel
              data[i] = data[i] / div * div + div / 2;
          }
      }
}



Mat derivative(Mat im){
  Mat out(im.rows-1,im.cols,0);

  for(int i = 0; i < im.rows-1; i++){
    out.at<double>(i,0) = im.at<double>(i,0)-im.at<double>(i+1,0);
  }
  return out;
}

int* line_on_hor(Mat image, int hor){
  Mat line;Mat imHsv;

  cvtColor(image,imHsv,CV_BGR2HSV);

Rect r(0,hor,image.size().width, 1);
Mat roi = imHsv(r);

Mat der = derivative(roi);


return (int*)calloc(2,sizeof(int));
}

double sumA(double *a, int size){
  double val = 0.0;

  for(int i = 0; i < size; i++){
    val += a[i];
  }
  return val;
}

int minVal(double a[], int size){
  double val = sumA(a,size)+1;
  int index;

  for(int i = 0; i < size; i++){
    if(val > a[i]){
      index = i;
      val = a[i];
    }
  }
  return val;
}

int maxVal(double a[], int size){
  double val = 0.0;
  int index;

  for(int i = 0; i < size; i++){
    //printf("%.2f\t", a[i]);
    if(val < a[i]){
      index = i;
      val = a[i];
    }
  }
  //printf("\n");
  return val;
}


void norma(double a[], int size){
  double min = minVal(a,size);
  printf("min %f\n", min);
  double max = maxVal(a,size);
  printf("max %f\n", max);
  for(int i = 0; i < size; i++){
    printf("%f\t", a[i]);
      a[i] = (a[i] - min)/(max-min);
      printf("%f\n", a[i]);
  }
}

int maxIndex(double a[], int size){
  double val = 0.0;
  int index;

  for(int i = 0; i < size; i++){
    if(val < a[i]){
      index = i;
      val = a[i];
    }
  }
  return index;
}

int calColor(Mat image, int parts){
  int part[parts+1];
  double val[parts];
  int width = floor(image.cols/parts);
  part[0] = 0;
  for(int i = 1;i <= parts;i++){
    part[i] = i*width;
  }

  vector<Mat> bgr_planes;
  split( image, bgr_planes );

  for(int i = 0; i < parts;i++){
    for(int j = part[i]; j < part[i+1];j++){
      for(int k = 0; k < image.rows; k++){
        val[i] += bgr_planes[0].at<double>(k,j) + bgr_planes[1].at<double>(k,j) + bgr_planes[2].at<double>(k,j);
        //printf("max: %d %d Test %d %d %d\n",bgr_planes[0].rows,image.rows,i,j,k);
      }
    }

  }

  norma(val,parts);
  int max = maxIndex(val,parts);

  printf("%d\n", max);

  return -(((parts+1)/2)-max);

}










































int** dominantColors(Mat image, int histSize){
  Mat hist;
int nimages = 1; // Only 1 image, that is the Mat scene.
int channels[] = {0}; // Index for hue channel
int dims = 1; // Only 1 channel, the hue channel
int hs =9;
int histS[] = {hs}; // 9 bins, 1 each for Red, RY, Yellow, YG etc.
float hranges[] = { 0, 180 }; // hue varies from 0 to 179, see cvtColor
const float *rang[] = {hranges};

// Compute the histogram.
calcHist(&image,
nimages,
channels,
Mat(), // No mask
hist, dims, histS, rang, true);

/// Draw the histogram
  int w = 400; int h = 400;
  int bin_w = cvRound( (double) w / hs );
  Mat histImg = Mat::zeros( w, h, CV_8UC3 );

  for( int i = 0; i < 25; i ++ )
     { rectangle( histImg, Point( i*bin_w, h ), Point( (i+1)*bin_w, h - cvRound( hist.at<float>(i)*h/255.0 ) ), Scalar( 0, 0, 255 ), -1 ); }

  imshow( "Histogram", histImg );

// Now hist will contain the counts in each bin.
// Lets just print out the values. Note that you can output Mat using std::cout
cout << "Histogram: " << endl << hist << endl;

// To access the individual bins, you can either iterate over them
// or use hist.at<uchar>(i, j); Note that one of the index should be 0
// because hist is 1D histogram. Print out hist.rows and hist.cols to see if hist is a N x 1 or 1 x N matrix.
/*
MatIterator_<uchar> it, end;
int binIndex = 0;
for( it = hist.begin<uchar>(), end = hist.end<uchar>(); it != end; ++it)
{
  printf("Count in %d bin: %d\n", binIndex, *it);
  ++binIndex;
}*/

return (int**)calloc(2,sizeof(int*));
}



/*/// Separate the image in 3 places ( B, G and R )
int number = 10;
  vector<Mat> bgr_planes;
  split( image, bgr_planes );

  /// Set the ranges ( for B,G,R) )
  float range[] = { 0, 256 } ;
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;

  Mat b_hist, g_hist, r_hist;

  /// Compute the histograms:
  calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &number, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &number, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &number, &histRange, uniform, accumulate );

  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/number );

  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );


  /// Draw for each channel
  for( int i = 1; i < number; i++ )
  {
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
  }

  /// Display
  namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
  imshow("calcHist Demo", histImage );

  //waitKey(0);
  return (int**)calloc(2,sizeof(int*));

}
*/
