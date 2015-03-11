#include <stdio.h>
#include <iostream>
#include <ctime>
#include <cstdio> 

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "ros/ros.h"

using namespace cv;
using namespace std;


cv_bridge::CvImageConstPtr cv_ptr;
Mat frame;


void callback(const sensor_msgs::ImageConstPtr& msg) 
{

  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8); 
    frame = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
        
}

int main( int argc, char** argv )
{


  std::time_t rawtime;                                                      
  std::tm* timeinfo;                                                        
  char buffer [80];                                                         
  std::time(&rawtime);                                                      
  timeinfo = std::localtime(&rawtime);                                      
  std::strftime(buffer, 80, "%Y-%m-%d", timeinfo);                          
  std::puts(buffer);                                                        
  std::string str(buffer);                                                  

  Mat img_object = imread("/home/bwi/Desktop/template_" + str + ".jpg", CV_LOAD_IMAGE_GRAYSCALE );



  cv_ptr.reset (new cv_bridge::CvImage);

  // CvCapture* capture; 
  //
  // capture = cvCaptureFromCAM(0); 
  //
  // if( !img_object.data ) {
  //   std::cout<< "Error reading images " << std::endl; 
  //   return -1;
  // }

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;
  SurfFeatureDetector detector( minHessian );
  std::vector<KeyPoint> keypoints_object, keypoints_frame;

  detector.detect( img_object, keypoints_object );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;
  Mat descriptors_object, descriptors_frame;

  extractor.compute( img_object, keypoints_object, descriptors_object );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;


  // if (!capture) {
  //   printf("No camera detected! ");
  //   exit(1);
  // }

  namedWindow("WindowName", CV_WINDOW_AUTOSIZE);

  ros::init(argc, argv, "object_detection");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/nav_kinect/rgb/image_color", 1, callback);

  int cnt = 0;
  ros::Rate r(10);
  while (ros::ok()) {

    r.sleep();
    ros::spinOnce();

    // frame = cvQueryFrame(capture);

    if (frame.empty()) {
      printf("No captured frame -- Break!");
      continue;
      // break;
    }


    detector.detect(frame, keypoints_frame);
    extractor.compute(frame, keypoints_frame, descriptors_frame);
    matcher.match(descriptors_object, descriptors_frame, matches);

    //-- Quick calculation of max and min distances between keypoints
    double max_dist = 0; double min_dist = 100;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }
    // printf("-- Max dist : %f \n", max_dist );
    // printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 3*min_dist )
       { good_matches.push_back( matches[i]); }
    }

    Mat img_matches;
    drawMatches( img_object, keypoints_object, frame, keypoints_frame,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    // printf("%d\n", good_matches.size());

    for( int i = 0; i < good_matches.size(); i++ )
    {
      //-- Get the keypoints from the good matches
      obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
      scene.push_back( keypoints_frame[ good_matches[i].trainIdx ].pt );
    }

    if (obj.size() < 4 || scene.size() < 4) 
      continue;

    Mat H = findHomography( obj, scene, CV_RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

    float dis = 0.0;
    dis += norm( (scene_corners[0] + Point2f( img_object.cols, 0)) - (scene_corners[1] + Point2f( img_object.cols, 0)) );
    dis += norm( (scene_corners[1] + Point2f( img_object.cols, 0)) - (scene_corners[2] + Point2f( img_object.cols, 0)) );
    dis += norm( (scene_corners[2] + Point2f( img_object.cols, 0)) - (scene_corners[3] + Point2f( img_object.cols, 0)) );
    dis += norm( (scene_corners[3] + Point2f( img_object.cols, 0)) - (scene_corners[0] + Point2f( img_object.cols, 0)) );
   
    //-- Show detected matches

    if (dis > 500)
      cnt++;
    else
      cnt = 0;

    if (cnt >= 5) 
    {
      imwrite("/home/bwi/Desktop/logo_" + str + ".jpg", frame);
      break;
    }


    imshow( "WindowName", img_matches );
    waitKey(1);
  }

  return 0;
}

