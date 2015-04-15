
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

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

#include "bwi_scavenger/Whiteboard.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

sensor_msgs::ImageConstPtr image; 

std::string directory, file; 
std::string default_dir = "/home/bwi/shiqi/";

enum Status {RUNNING, DONE};

Status s = RUNNING; 

struct my_pose {
    float x; 
    float y;
} m, a1, b1, c1, d1, a2, b2, c2, d2; 

/*
    a           b
       m
    c           d
    -------------
1, board near conference room
2, board near 400/500 doors
*/
    // M of coordinates(x,y) is inside the rectangle iff, 
    // (0 < AM dot AB < AB dot AB) AND (0 < AM dot AC < AC dot AC)

bool inRectangle(my_pose* m, my_pose* a, my_pose* b, my_pose* c) 
{
    
    float am_ab, ab_ab, am_ac, ac_ac;
    am_ab = (m->x - a->x) * (b->x - a->x) + (m->y - a->y) * (b->y - a->y);
    ab_ab = (b->x - a->x) * (b->x - a->x) + (b->y - a->y) * (b->y - a->y);
    am_ac = (m->x - a->x) * (c->x - a->x) + (m->y - a->y) * (c->y - a->y);
    ac_ac = (c->x - a->x) * (c->x - a->x) + (c->y - a->y) * (c->y - a->y);

    if (0 < am_ab && am_ab < ab_ab && 0 < am_ac && am_ac < ac_ac)
        return true;
    else
        return false;
}

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // // for old map
    // a1.x = 62.67;
    // a1.y = 9.89;
    // b1.x = 62.27;
    // b1.y = 4.27;
    // c1.x = 60.85;
    // c1.y = 9.36;
    // 
    // a2.x = 39.73;
    // a2.y = 17.12;
    // b2.x = 39.80;
    // b2.y = 11.17;
    // c2.x = 38.24;
    // c2.y = 16.94;

    // for new map
    a1.x = -30.88;
    a1.y = 0.06;
    b1.x = -29.48;
    b1.y = 0.06;
    c1.x = -30.89;
    c1.y = -3.16;
    
    a2.x = -8.25;
    a2.y = -5.99;
    b2.x = -6.83;
    b2.y = -6.05;
    c2.x = -8.21;
    c2.y = -11.26;
    
    geometry_msgs::PoseStampedConstPtr poseStamped = msg; 

    m.x = poseStamped->pose.position.x;
    m.y = poseStamped->pose.position.y; 


    if (inRectangle(&m, &a1, &b1, &c1) || inRectangle(&m, &a2, &b2, &c2)) 
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
        cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);

        file = directory + "/whiteboard-" + str + ".jpg"; 
        cv::imwrite(file, cv_ptr->image);
        s = DONE;

    }
}

// keep saving the most recent image
void callback_image_saver(const sensor_msgs::ImageConstPtr& msg)
{
  image = msg; 
}

bool whiteboard_search(bwi_scavenger::Whiteboard::Request &req, 
    bwi_scavenger::Whiteboard::Response &res) {

    ros::Rate r(10);
    while (s != DONE && ros::ok()) {
        
        ros::spinOnce(); 
        
    }
    res.path_to_image = file; 
    return true;
    
}

// main function of the node
int main(int argc, char ** argv)
{

    ros::init(argc, argv, "whiteboard_server");
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe(
                    "/segbot_pcl_person_detector/human_poses", 100, callback); 

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub2 = it.subscribe(
                    "/nav_kinect/rgb/image_color", 1, callback_image_saver);


    ROS_INFO("\nPath to saved files can be specified via private parameter: directory\n"); 
    ROS_INFO("File name: whiteboard-yyyy-mm-dd\n");

    ros::param::param<std::string>("~directory", directory, default_dir);


    ros::ServiceServer service = nh.advertiseService ("whiteboard_service", 
        whiteboard_search);

    ros::Rate r(10); 

    while (ros::ok()) {
        
        ros::spinOnce(); 
        r.sleep();

    }
    return true; 
}




