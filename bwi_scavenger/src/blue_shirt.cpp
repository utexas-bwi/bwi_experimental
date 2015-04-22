
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

ros::NodeHandle * nh;
sensor_msgs::ImageConstPtr image; 

// shirt_color is a string saving the color of shirt
std::string shirt_color, directory, file;

std::string default_dir = "/home/bwi/shiqi/";

bool color_shirt_detected; 

struct Rgb {
    float r;
    float g;
    float b;        
    Rgb() : r(), g(), b() {}
    Rgb( float rr, float gg, float bb) : r(rr), g(gg), b(bb) {}
} red, blue, green, yellow;


void callback_image_saver(const sensor_msgs::ImageConstPtr& msg) {image = msg;} 

float get_color_dis(const pcl::PointXYZRGB *c1, Rgb *c2) {
    return pow(pow(c1->r- c2->r, 2.0) + pow(c1->g - c2->g, 2.0) + 
        pow(c1->b - c2->b, 2.0), 0.5);
}

void callback_human_detection(const PointCloud::ConstPtr& msg)
{
    float COLOR_RATIO = 0.35;
    float max_y = -10000.0;
    float min_y = 10000.0; 
    float DISTANCE_TO_COLOR = 200;

    int pixel_cnt = 0;
    int color_cnt = 0; 
    float dis = 0.0;

    red = Rgb(255.0, 0.0, 0.0);
    blue = Rgb(0.0, 0.0, 255.0);
    green = Rgb(0.0, 255.0, 0.0);
    yellow = Rgb(255.0, 255.0, 0.0);

    BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points) {
        pixel_cnt++;

        // here we assume the waist height is 90cm, and neck height is 160cm
        // the robot sensor's height is 60cm
        max_y = (pt.y > max_y) ? pt.y : max_y;
        min_y = (pt.y < min_y) ? pt.y : min_y; 

        if (pt.y > -0.9 && pt.y < -0.1) {

            if (shirt_color.compare("red") == 0)
                dis = get_color_dis( &pt, &red); 
            else if (shirt_color.compare("blue") == 0)
                dis = get_color_dis( &pt, &blue); 
            else if (shirt_color.compare("green") == 0)
                dis = get_color_dis( &pt, &green); 
            else if (shirt_color.compare("yellow") == 0)
                dis = get_color_dis( &pt, &yellow); 
            else
                ROS_ERROR("parameter of ~shirt_color: error\n");

            if (dis < DISTANCE_TO_COLOR)
                color_cnt++;
        }
    }

    float ratio = (float) color_cnt / (float) pixel_cnt;
    ROS_INFO("%s: ratio is %f\n", ros::this_node::getName().c_str(), ratio);

    // when color-shirt person detected
    if (ratio > COLOR_RATIO && ros::ok()) { 

        ROS_INFO("%s: person wearing %s shirt detected\n",
            ros::this_node::getName().c_str(), shirt_color.c_str()); 
        
        cv_bridge::CvImageConstPtr cv_ptr = 
            cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);

        cv::imwrite(file, cv_ptr->image);

        // this global variable decides when the color-shirt service returns
        color_shirt_detected = true; 
        ros::Duration(5.0).sleep(); 

    } else {
        color_shirt_detected = false;
    }
}

bool find_color_shirt(bwi_scavenger::ColorShirt::Request &req, 
    bwi_scavenger::ColorShirt::Response &res) {
 
    ros::Subscriber sub1 = nh->subscribe
        ("/segbot_pcl_person_detector/human_clouds", 1, 
        callback_human_detection);

    switch ( (int) req.color) {
        
        case 1:
            shirt_color = "red"; break;
        case 2:
            shirt_color = "blue"; break;
        case 3:
            shirt_color = "green"; break;
        case 4:
            shirt_color = "yellow"; break;
        default:
            ROS_ERROR("%s: color not recognized",
                ros::this_node::getName().c_str()); 
            
    }

    // blocks here until service_status becomes DONE
    while ( !color_shirt_detected && ros::ok() ) 
        ros::Duration(0.1).sleep();

    // service done, and return the path to the image
    res.path_to_image = file; 
    return true;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blue_shirt_server");
    nh = new ros::NodeHandle();

    // directory to save files, used for service results
    ros::param::param<std::string>("~directory", directory, default_dir);
    file = directory + "shirt.jpg"; 

    ros::ServiceServer service = nh->advertiseService("blue_shirt_service", 
        find_color_shirt);


    image_transport::ImageTransport it(*nh);
    image_transport::Subscriber sub2 = it.subscribe 
        ("/nav_kinect/rgb/image_color", 1, callback_image_saver);

    ros::Duration(1.0).sleep();

    // 3 threads: receiving point cloud, receiving image, color-shirt service
    ros::AsyncSpinner spinner(3); 
    spinner.start(); 
    ros::waitForShutdown(); 

    return true;

}


