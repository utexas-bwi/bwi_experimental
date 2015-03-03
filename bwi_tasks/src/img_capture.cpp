#include <stdio.h>
#include <iostream>
#include <ctime>
#include <cstdio>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
  if (argc != 1)
    return -1;

  CvCapture* capture; 

  capture = cvCaptureFromCAM(0); 

  if (!capture) {
    printf("No camera detected! ");
    return -1;
  }

  namedWindow("WindowName", CV_WINDOW_AUTOSIZE);
  Mat frame;

  while (true) {

    frame = cvQueryFrame(capture);

    if (frame.empty()) {
      printf("No captured frame -- Break!");
      break;
    }

    imshow( "WindowName", frame);
    char input = waitKey(10);

    if (input == 's') {
      std::time_t rawtime;
      std::tm* timeinfo;
      char buffer [80];
      
      std::time(&rawtime);
      timeinfo = std::localtime(&rawtime);
      std::strftime(buffer, 80, "%Y-%m-%d", timeinfo);
      std::puts(buffer);
      std::string str(buffer);

      imwrite("/home/bwi/Desktop/template_" + str + ".jpg", frame); 
      break;
    }
  }

  return 0;
  }

