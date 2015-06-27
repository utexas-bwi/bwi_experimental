
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "ScavTaskColorShirt.h"


#define INTMAX (32767)
#define INTMIN (-32767)
#define COLOR_RATIO (0.35)
#define DISTANCE_TO_COLOR (200)
#define SHIRT_HEIGHT_TOP (-0.1)
#define SHIRT_HEIGHT_BOTTOM (-0.9)

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

sensor_msgs::ImageConstPtr image; 

std::string path_to_image; 

Rgb baseline; 

void callback_image_saver(const sensor_msgs::ImageConstPtr& msg) {
    image = msg;
} 

void callback_human_detection(const PointCloud::ConstPtr& msg)
{

    int color_cnt = 0; 

    switch (shirt_color) {
        case RED:       baseline = Rgb(255.0, 0.0, 0.0);    break;
        case BLUE:      baseline = Rgb(0.0, 0.0, 255.0);    break;
        case GREEN:     baseline = Rgb(0.0, 255.0, 0.0);    break;
        case YELLOW:    baseline = Rgb(255.0, 255.0, 0.0);  break; 
        case ORANGE:    baseline = Rgb(191.0, 87.0, 0.0);   break;
    }

    BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points) {

        // here we assume the waist height is 90cm, and neck height is 160cm; 
        // the robot sensor's height is 60cm
        if (pt.y > SHIRT_HEIGHT_BOTTOM and pt.y < SHIRT_HEIGHT_TOP 
                and getColorDistance( &pt, &baseline) < DISTANCE_TO_COLOR) 
            color_cnt++;
    }

    float ratio = (float) color_cnt / (float) msg->points.size();

    ROS_INFO("ratio is %f", ratio);

    if (ratio > COLOR_RATIO && ros::ok()) { 

        ROS_INFO("person wearing %s shirt detected", shirt_color.c_str()); 
        
        boost::posix_time::ptime curr_time = boost::posix_time::second_clock::local_time();  
        std::string time_str = boost::posix_time::to_simple_string(curr_time); 

        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
        
        if (false == boost::filesystem::is_directory(directory)) {
            boost::filesystem::path tmp_path(directory);
            boost::filesystem::create_directory(tmp_path);
        }
        path_to_image = directory + "color_shirt_" + time_str; 
        cv::imwrite(path_to_image, cv_ptr->image);
    }
}

ScavTaskColorShirt::ScavTaskColorShirt(ros::NodeHandle *nh, std::string dir, Color shirt_color) {
    this->nh = nh; 
    directory = dir; 
    color = shirt_color; 
    task_description = "find a person wearing a color shirt: "; 
}

void ScavTaskColorShirt::motionThread() {

    std::string path_to_yaml = ros::package::getPath("bwi_scavenger") + "/support/real.yaml";
    planner = new SearchPlanner(nh, path_to_yaml, 0.2);           

    int next_goal_index;                                                        
    while (ros::ok()) {
        planner->moveToNextScene( planner->selectNextScene(planner->belief, next_goal_index) );
        planner->analyzeScene(0.25*PI, PI/10.0);
        planner->updateBelief(next_goal_index);
    }
}

void ScavTaskColorShirt::visionThread() {

    ros::Subscriber sub1 = nh->subscribe("/segbot_pcl_person_detector/human_clouds", 1, callback_human_detection);

    image_transport::ImageTransport it(*nh);
    image_transport::Subscriber sub = it.subscribe ("/nav_kinect/rgb/image_color", 1, callback_image_saver);

    ros::Rate rate(10); 

    while (ros::ok() and rate.sleep()) {
        ros::spinOnce(); 
    }
}

void ScavTaskColorShirt::executeTask(int timeout, TaskResult &result, std::string &record)
{

    boost::thread motion(this->motionThread); 
    boost::thread vision(this->visionThread);

    motion.join();
    vision.join();

    record = path_to_image; 
    result = SUCCEEDED; 
}


