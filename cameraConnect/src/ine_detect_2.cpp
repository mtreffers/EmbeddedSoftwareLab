#include <iostream>
#include <vector>
#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

void own_threshold(Mat *image, int thresh){
  for(int i = 0; i < image->cols; i++){
    for(int j = 0; j < image->rows; j++){
      Vec3b col = image->at<int>(i,j);
      if(col[0] > thresh && col[1] > thresh && col[2] > thresh){
            col[0] = 255;
            col[1] = 255;
            col[2] = 255;
            image->at<int>(i,j) = col;
      }
    }
  }
}

int* control_robot(Mat image, int thresh){

}
