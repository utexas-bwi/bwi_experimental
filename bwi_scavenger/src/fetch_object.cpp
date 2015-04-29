
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include "bwi_scavenger/FetchObject.h"
#include "bwi_kr_execution/ExecutePlanAction.h"
#include "bwi_msgs/QuestionDialog.h"


ros::NodeHandle *nh; 

std::string default_dir = "/home/bwi/shiqi/";

bool callback(bwi_scavenger::FetchObject::Request &req, 
    bwi_scavenger::FetchObject::Response &res) {

    std::string room_from, room_to, object_name, file; 
    ROS_INFO("fetch_object_service request received"); 
    
    ros::ServiceClient gui_service_client = nh->serviceClient
        <bwi_msgs::QuestionDialog> ("question_dialog"); 
    bwi_msgs::QuestionDialog srv;
    
    // specify the task 
    srv.request.type = 1; 
    srv.request.message = "I am working on a 'fetch object' task. Please click the button, if you can help with specifying the task"; 
    srv.request.options = std::vector <std::string> (1, "button");
    gui_service_client.call(srv); 

    ROS_INFO("fetch_object_service: task specified"); 

    // what's the object's name? saved to room_from
    srv.request.type = 2;
    srv.request.message = "What is the object's name?"; 
    gui_service_client.call(srv);
    object_name = srv.response.text;

    // where to get the object, saved to room_from
    srv.request.type = 2;
    srv.request.message = "Where to get the object? E.g., l3_420."; 
    gui_service_client.call(srv);
    room_from = srv.response.text;

    // where to deliver the object, saved to room_to
    srv.request.type = 2;
    srv.request.message = "Where to deliver the object? E.g., l3_420."; 
    gui_service_client.call(srv);
    room_to = srv.response.text;

    actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> 
        asp_plan_client("action_executor/execute_plan", true); 
    asp_plan_client.waitForServer();

    // print to gui: moving to the first room
    srv.request.type = 0;
    srv.request.message = "Moving to room: " + room_from; 
    gui_service_client.call(srv);

    // wrapping up a "goal" for moving the robot platform
    bwi_kr_execution::ExecutePlanGoal goal;
    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;

    // move to the room to ask for the object
    fluent.name = "not at";
    fluent.variables.push_back(room_from);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);
    ROS_INFO("sending goal");
    asp_plan_client.sendGoalAndWait(goal);

    if (asp_plan_client.getState() == 
        actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Succeeded!");
    else
        ROS_INFO("Failed!");

    // request to load object
    srv.request.type = 1; 
    srv.request.message = "Please help me load object: " + object_name; 
    srv.request.options = std::vector <std::string> (1, "Loaded");
    gui_service_client.call(srv); 

    // print to gui: moving to the second room
    srv.request.type = 0;
    srv.request.message = "Moving to room: " + room_to; 
    gui_service_client.call(srv);

    // move to the second room for delivery
    fluent.variables.clear(); 
    rule.body.clear();
    goal.aspGoal.clear(); 

    fluent.name = "not at";
    fluent.variables.push_back(room_to);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);
    ROS_INFO("sending goal");
    asp_plan_client.sendGoalAndWait(goal);

    if (asp_plan_client.getState() == 
        actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Succeeded!");
    else
        ROS_INFO("Failed!");

    // request to unload object
    srv.request.type = 1; 
    srv.request.message = "Please help me unload object: " + object_name; 
    srv.request.options = std::vector <std::string> (1, "Unloaded");
    gui_service_client.call(srv); 

    ROS_INFO("fetch_object_service task done"); 

    std::ofstream fs;
    file = default_dir + "fetch_object_log.txt";
    fs.open(file.c_str()); 
    fs << room_from + "\n" + object_name + "\n" + room_to + "\n"; 
    fs.close(); 

    res.path_to_log = file; 

    return true; 
}


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "fetch_object_server");
    nh = new ros::NodeHandle(); 
    
    ros::ServiceServer service = nh->advertiseService("fetch_object_service", 
        callback); 
    ROS_INFO("fetch_object_service ready"); 
    ros::spin(); 

    return 0;    
}
