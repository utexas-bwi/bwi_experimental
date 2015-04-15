
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <bwi_scavenger/ColorShirt.h>

#include <iostream>
#include <cstdio>
#include <ctime>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

sensor_msgs::ImageConstPtr image; 

// shirt_color is a string saving the color of shirt
std::string shirt_color, directory, file;

std::string default_dir = "/home/bwi/shiqi/";

enum Status {RUNNING, DONE}; 

Status s; 

struct rgb {
    float r;
    float g;
    float b;        
} red, blue, green, yellow;


void callback_image_saver(const sensor_msgs::ImageConstPtr& msg)
{
    image = msg; 
    if (s == DONE)
        return; 
}

void callback_human_detection(const PointCloud::ConstPtr& msg)
{
    float COLOR_RATIO = 0.25;
    float max_y = -10000.0;
    float min_y = 10000.0; 
    float DISTANCE_TO_COLOR = 200;

    int cnt = 0;
    int color_cnt = 0; 
    float dis = 0.0;

    red.r = 255.0;
    red.g = 0.0;
    red.b = 0.0;
    blue.r = 0.0;
    blue.g = 0;
    blue.b = 255.0;
    green.r = 0.0;
    green.g = 255.0;
    green.b = 0.0;
    yellow.r = 255.0;
    yellow.g = 255.0;
    yellow.b = 0.0;


    BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points) {
        cnt++;

        // here we assume the waist height is 90cm, and neck height is 160cm
        // the robot sensor's height is 60cm
        if (pt.y > max_y)
            max_y = pt.y;

        if (pt.y < min_y)
            min_y = pt.y;

        if (pt.y > -0.9 && pt.y < -0.1) {

            if (shirt_color.compare("red") == 0)
                dis = pow(pow(pt.r- red.r, 2.0) + pow(pt.g - red.g, 2.0) + 
                    pow(pt.b - red.b, 2.0), 0.5);

            else if (shirt_color.compare("blue") == 0)
                dis = pow(pow(pt.r- blue.r, 2.0) + pow(pt.g - blue.g, 2.0) + 
                    pow(pt.b - blue.b, 2.0), 0.5);

            else if (shirt_color.compare("green") == 0)
                dis = pow(pow(pt.r- green.r, 2.0) + pow(pt.g - green.g, 2.0) + 
                    pow(pt.b - green.b, 2.0), 0.5);

            else if (shirt_color.compare("yellow") == 0)
                dis = pow(pow(pt.r- yellow.r, 2.0) + pow(pt.g - yellow.g, 2.0) + 
                    pow(pt.b - yellow.b, 2.0), 0.5);

            else
                ROS_ERROR("parameter of ~shirt_color: error\n");

            if (dis < DISTANCE_TO_COLOR)
                color_cnt++;
        }
    }

    float ratio = (float)color_cnt/(float)cnt;
    ROS_INFO("ratio: %f\n", ratio);

    if (ratio > COLOR_RATIO) { 

        ROS_INFO("person with color shirt detected \n"); 
        
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

            ros::param::get("~directory", directory);
            file = directory + "/shirt_" + str + ".jpg"; 
            cv::imwrite(file, cv_ptr->image);
            s = DONE; 

            return; 
        } 
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    } 
}

bool find_color_shirt(bwi_scavenger::ColorShirt::Request &req, 
    bwi_scavenger::ColorShirt::Response &res) {
 
    switch (req.color) {
        
        case 1:
            shirt_color = "red";
            break;
        case 2:
            shirt_color = "blue";
            break;
        case 3:
            shirt_color = "green";
            break;
        case 4:
            shirt_color = "yellow"; 
            break;
        default:
            ROS_ERROR("service call to color_shirt error"); 
            
    }

    // directory to save files
    ros::param::param<std::string>("~directory", directory, default_dir);


    while (s != DONE) {
        ros::spinOnce();
    }

    res.path_to_image = file; 
    return true;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blue_shirt_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("blue_shirt_service", 
        find_color_shirt);

    ros::Subscriber sub1 = nh.subscribe
        ("/segbot_pcl_person_detector/human_clouds", 1, 
        callback_human_detection);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub2 = it.subscribe 
        ("/nav_kinect/rgb/image_color", 1, callback_image_saver);

    // ros::Duration(1.0).sleep();
    ros::Rate r(10);
    ros::spin();

    return true;

}




