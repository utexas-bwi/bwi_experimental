
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <fstream>
#include <boost/filesystem.hpp>

#include "ScavTaskFetchObject.h"
#include "bwi_kr_execution/ExecutePlanAction.h"
#include "bwi_msgs/QuestionDialog.h"


ScavTaskFetchObject::executeTask(int timeout, TaskResult &result, std::string &record) {
    
    gui_service_client = new ros::ServiceClient(nh->serviceClient <bwi_msgs::QuestionDialog> ("question_dialog")); 

    ros::Duration(0.5).sleep(); 
    bwi_msgs::QuestionDialog srv;
    
    // specify the task 
    srv.request.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION; 
    srv.request.message = "Please click the button, if you can help with specifying the 'fetch object' task"; 
    srv.request.options = std::vector<std::string>(1, "button");
    srv.request.timeout = bwi_msgs::QuestionDialogRequest::NO_TIMEOUT; 
    gui_service_client->waitForExistence(); 
    gui_service_client->call(srv); 

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

    actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> asp_plan_client("/action_executor/execute_plan", true); 
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

    ROS_INFO(asp_plan_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ? "Succeeded!" : "Failed");

    // request to load object
    srv.request.type = 1; 
    srv.request.message = "Please help me load object: " + object_name; 
    srv.request.options = std::vector <std::string> (1, "Loaded"); 
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

    ROS_INFO(asp_plan_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ? "Succeeded!" : "Failed!");

    // request to unload object
    srv.request.type = 1; 
    srv.request.message = "Please help me unload object: " + object_name; 
    buttons = std::vector <std::string> (1, "Unloaded"); 
    srv.request.options = buttons;
    gui_service_client->call(srv); 

    boost::posix_time::ptime curr_time = boost::posix_time::second_clock::local_time(); 
    std::string path_to_image = directory;
    path_to_image += "fetch_object_" + boost::posix_time::to_simple_string(curr_time); 

    if (false == boost::filesystem::is_directory(directory)) {
        boost::filesystem::path tmp_path(directory); 
        boost::filesystem::create_directory(tmp_path); 
    } else {
        std::ofstream fs;
        fs.open(path_to_image.c_str(), std::ofstream::app); 
        fs << "origin room: " << room_name_from + "\n" 
            << "object name: " << object_name + "\n" 
            << "destination room: "+ room_name_to + "\n"; 
        fs.close(); 
    }

    ROS_INFO("fetch_object_service task done"); 

    record = path_to_image;
    result = SUCCEEDED; 

}
