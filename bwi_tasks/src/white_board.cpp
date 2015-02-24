
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

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

struct my_pose {
    float x; 
    float y;
} p1, p2; 

/*
    a           b
       p1   p2
    c           d
*/

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    p1.x = 61.099;
    p1.y = 7.479;
    p2.x = 60.981;
    p2.y = 6.310;
    float range= 1.0;

    geometry_msgs::PoseStampedConstPtr poseStamped = msg; 

    float x = poseStamped->pose.position.x;
    float y = poseStamped->pose.position.y; 

    float dis_p1 = pow(pow(p1.x-x, 2.0) + pow(p1.y - y, 2.0), 0.5); 
    float dis_p2 = pow(pow(p2.x-x, 2.0) + pow(p2.y - y, 2.0), 0.5); 

    if (dis_p1 < 1.0 || dis_p2 < 1.0) 
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

        cv_bridge::CvImageConstPtr cv_ptr;
        try 
        {
            cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
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

