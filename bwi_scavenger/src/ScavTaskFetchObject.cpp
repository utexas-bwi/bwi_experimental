
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>

#include "bwi_scavenger/FetchObject.h"
#include "bwi_kr_execution/ExecutePlanAction.h"
#include "bwi_msgs/QuestionDialog.h"


std::string default_dir = "/home/bwi/shiqi/";

ros::ServiceClient * gui_service_client; 

bool callback(bwi_scavenger::FetchObject::Request &req, 
    bwi_scavenger::FetchObject::Response &res) {

    std::string room_from, room_to, object_name, file; 
    std::vector <std::string> buttons;

    ROS_INFO("fetch_object_service request received"); 
    
    ros::Duration(0.5).sleep(); 
    bwi_msgs::QuestionDialog srv;
    
    // specify the task 
    srv.request.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION; 
    srv.request.message = "Please click the button, if you can help with specifying the 'fetch' task"; 
    buttons.push_back("button"); 
    srv.request.options = buttons;
    srv.request.timeout = bwi_msgs::QuestionDialogRequest::NO_TIMEOUT; 
    gui_service_client->waitForExistence(); 
    gui_service_client->call(srv); 

    ROS_INFO("fetch_object_service: task specified"); 

    // what's the object's name? saved to room_from
    srv.request.type = 2;
    srv.request.message = "What is the object's name?"; 
    gui_service_client->call(srv);
    object_name = srv.response.text;

    // where to get the object, saved to room_from
    srv.request.type = 2;
    srv.request.message = "Where to get the object? E.g., l3_420."; 
    gui_service_client->call(srv);
    room_from = srv.response.text;

    // where to deliver the object, saved to room_to
    srv.request.type = 2;
    srv.request.message = "Where to deliver the object? E.g., l3_420."; 
    gui_service_client->call(srv);
    room_to = srv.response.text;

    actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> 
        asp_plan_client("/action_executor/execute_plan", true); 
    asp_plan_client.waitForServer();

    // print to gui: moving to the first room
    srv.request.type = 0;
    srv.request.message = "Moving to room: " + room_from; 
    gui_service_client->call(srv);

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
    buttons.clear();
    buttons = std::vector <std::string> (1, "Loaded"); 
    srv.request.options = buttons;
    gui_service_client->call(srv); 

    // print to gui: moving to the second room
    srv.request.type = 0;
    srv.request.message = "Moving to room: " + room_to; 
    gui_service_client->call(srv);

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
    buttons = std::vector <std::string> (1, "Unloaded"); 
    srv.request.options = buttons;
    gui_service_client->call(srv); 

    ROS_INFO("fetch_object_service task done"); 

    std::ofstream fs;
    if (boost::filesystem::is_directory(default_dir) == false)
        boost::filesystem::create_directory(default_dir); 
    file = default_dir + "fetch_object_log.txt";
    fs.open(file.c_str(), std::ofstream::app); 
    fs << room_from + "\n" + object_name + "\n" + room_to + "\n"; 
    fs.close(); 

    res.path_to_log = file; 

    return true; 
}


ScavTaskFetchObject::executeTask(int timeout, TaskResult &result, std::string &record) {
    
    gui_service_client = new ros::ServiceClient(nh->serviceClient <bwi_msgs::QuestionDialog> ("question_dialog")); 

    record = path_to_image;
    result = SUCCEEDED; 

}
