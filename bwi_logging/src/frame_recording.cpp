
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

ros::NodeHandle *nh_pt; 

ros::Time start; 
int cnt; 
double frame_rate; 

void callback(const sensor_msgs::ImageConstPtr& rgb_img, 
    const sensor_msgs::ImageConstPtr& dep_img)
{
    std::ostringstream os; 
    ros::Duration diff = ros::Time::now() - start; 

    if (diff.toSec() < cnt * 1.0 / frame_rate) 
        return;
    
    cv_bridge::CvImageConstPtr rgb_pt, dep_pt;

    rgb_pt = cv_bridge::toCvShare(rgb_img, sensor_msgs::image_encodings::BGR8);
    dep_pt = cv_bridge::toCvShare(dep_img, sensor_msgs::image_encodings::BGR8); 

    std::string rgb_file, dep_file, rgb_sec, dep_sec; 
    os.str(""); os << cnt; 
    rgb_file = "rgb" + os.str() + ".jpg"; 
    dep_file = "depth" + os.str() + ".jpg"; 

    os.str(""); os << rgb_pt->header.stamp.sec; 
    rgb_sec = os.str();
    os.clear(); 
    os.str(""); os << dep_pt->header.stamp.sec; 
    dep_sec = os.str(); 

    cv::Mat rgb_frame = rgb_pt->image; 
    cv::Mat dep_frame = dep_pt->image; 

    cv::imwrite(rgb_file, rgb_frame); 
    cv::imwrite(dep_file, dep_frame); 
    
    ROS_INFO_STREAM(rgb_file << " saved with stamp.sec: " << rgb_sec); 
    ROS_INFO_STREAM(dep_file << " saved with stamp.sec: " << dep_sec); 

    cnt++; 
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "frame_recording_node");
    nh_pt = new ros::NodeHandle(); 

    cnt = 0; 
    ros::param::param <double> ("~frame_rate", frame_rate, 10.0);
    start = ros::Time::now(); 

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(*nh_pt,
        "/nav_kinect/rgb/image_raw", 1); 
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(*nh_pt, 
        "/nav_kinect/depth_registered/image_raw", 1);
    
    typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy; 

    message_filters::Synchronizer <MySyncPolicy> sync(MySyncPolicy(10),
        rgb_sub, depth_sub); 
    sync.registerCallback( boost::bind(&callback, _1, _2) ); 

    ros::spin(); 

    return 0; 
}
