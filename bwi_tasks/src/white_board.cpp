
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped"

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
#include <cstudio>
#include <ctime>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

sensor_msgs::ImageConstPtr image; 

struct my_pose {
    float x; 
    float y;
} a, b, c, d; 

/*
    a           b
        m
    c           d
*/

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    a.x = 59.9426460266;
    a.y = 5.25176143646;
    b.x = 60.0549316406;
    b.y = 9.36225509644;
    c.x = 61.6946525574;
    c.y = 5.01872062683;
    d.x = 61.9057197571;
    d.y = 9.26733779907; 

    my_pose pose = msg->pose; 

    am = (a.x - pose.Point.x) * (a.y - pose.Point.y); 
    ab = (a.x - b.x) * (a.y - b.y); 
    ad = (a.x - d.x) * (a.y - d.y); 

    if (am*ab > 0 && am*ab < ab*ab && am*ad > 0 && am*ad < ad*ad) 
    {
        std::time_t rawtime;
        std::tm* timeinfo;
        char buffer [80];

        std::time(&rawtime);
        timeinfo = std::localtime(&rawtime);
        std::strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
        std::puts(buffer);
        std::string str(buffer);

        ROS_INFO("People detected in frout of a white board, picture saved.");

        try 
        {
            cv_bridge::CvImageConstPtr cv_ptr;
            cv::imwrite("/home/bwi/Desktop/write_board_" + str + ".jpg",
                        cv_ptr->image);
        }
        catch (cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
}

// keep saving the most recent image
void callback_image_saver(const sensor_msgs::ImageConstPtr& msg)
{
  image = msg; 
}

// main function of the node
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "white_board");
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe(
                    "/segbot_pcl_person_detector/human_poses", 100, callback); 

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub2 = it.subscribe(
                    "/nav_kinect/rgb/image_color", 1, callback_image_saver);

    ros::spin(); 

    return 0; 
}

