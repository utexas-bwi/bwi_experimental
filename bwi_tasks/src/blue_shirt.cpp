
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <cstdio>
#include <ctime>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

sensor_msgs::ImageConstPtr image; 

void callback_image_saver(const sensor_msgs::ImageConstPtr& msg)
{
  image = msg; 
}

void callback_human_detection(const PointCloud::ConstPtr& msg)
{
  //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  int cnt = 0;
  int blue_cnt = 0; 
  float dis = 0.0;

  float max_y = -10000.0;
  float min_y = 10000.0; 
  BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points) {
    cnt++;

    // here we assume the waist height is 90cm, and neck height is 160cm
    // the robot sensor's height is 60cm
    if (pt.y > max_y)
      max_y = pt.y;

    if (pt.y < min_y)
      min_y = pt.y;

    if (pt.y > -0.9 && pt.y < -0.1) {
      // distance to BLUE (0, 0, 255)
      dis = pow(pow(pt.r-0.0, 2.0) + pow(pt.g-0.0, 2.0) + pow(pt.b-255.0, 2.0), 0.5);
      if (dis < 200)
        blue_cnt++;
    }
  }

  float ratio = (float)blue_cnt/(float)cnt;
  printf("ratio: %f\n", ratio);

  if (ratio > 0.25) { 

    ROS_INFO("person with BLUE SHIRT!!! \n"); 
    
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      std::time_t rawtime;
      std::tm* timeinfo;
      char buffer [80];

      std::time(&rawtime);
      timeinfo = std::localtime(&rawtime);
      std::strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
      std::puts(buffer);
      std::string str(buffer);

      cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
      cv::imwrite("/home/bwi/Desktop/blueshirt_" + str + ".jpg", cv_ptr->image);
    } 
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  } 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "segbot_blue_shirt");
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe<PointCloud>("/segbot_pcl_person_detector/human_clouds", 1, callback_human_detection);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub2 = it.subscribe("/nav_kinect/rgb/image_color", 1, callback_image_saver);

  ros::spin();
}
