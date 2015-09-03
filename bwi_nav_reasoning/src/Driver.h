#ifndef NAVEXECUTOR_H
#define NAVEXECUTOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include <yaml-cpp/yaml.h>

#include "StateAction.h"

class Driver {
public:
    Driver() {}
    Driver(ros::NodeHandle *, std::string); 

    std::string file_coordinates; 
    YAML::Node ynode; 
    geometry_msgs::PoseWithCovarianceStamped curr_position;
    int curr_row, curr_col; 
    float curr_x, curr_y; 

    ros::NodeHandle *nh; 

    ros::Publisher pub_simple_goal; 
    ros::Subscriber sub_amcl_pose; 
    
    void updateCurrentState(State &state); 
    bool moveToGoalState(const State &state); 

    void callbackUpdatePosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
}; 

#endif
