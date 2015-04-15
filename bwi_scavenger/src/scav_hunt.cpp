
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "bwi_msgs/QuestionDialog.h"
#include "bwi_scavenger/VisionTask.h"
#include "bwi_scavenger/Dialog.h"

#include <boost/lexical_cast.hpp>

#include <string>
#include <stdlib.h>
#include <vector>

std::string status_shirt, status_object, status_board;
std::string shirt_color, object_name, path_to_template;
std::string file_shirt, file_object, file_board, file_fetch, file_dialog; 

enum Status { TODO, DOING, DONE };


void task_shirt(ros::ServiceClient * client)
{


    bwi_scavenger::VisionTask srv;
    srv.request.type = 3;

    if (shirt_color.compare("red") == 0)
        srv.request.color = bwi_scavenger::VisionTaskRequest::RED; 
    else if (shirt_color.compare("blue") == 0)
        srv.request.color = bwi_scavenger::VisionTaskRequest::BLUE; 
    else if (shirt_color.compare("green") == 0)
        srv.request.color = bwi_scavenger::VisionTaskRequest::GREEN; 
    else if (shirt_color.compare("yellow") == 0)
        srv.request.color = bwi_scavenger::VisionTaskRequest::YELLOW; 

    client->call(srv); 
    file_shirt = srv.response.path_to_image; 
}

void task_object(ros::ServiceClient * client)
{

    ros::param::param <std::string> ("~path_to_template", path_to_template, 
        "/home/bwi/shiqi/template.jpg"); 
    ROS_INFO("path_to_template: %s", path_to_template.c_str()); 

    bwi_scavenger::VisionTask srv;
    srv.request.type = 2;
    srv.request.path_to_template = path_to_template; 
    client->call(srv); 
    file_object = srv.response.path_to_image; 
}

void task_board(ros::ServiceClient * client)
{

    ROS_INFO("whiteboard task"); 

    bwi_scavenger::VisionTask srv;
    srv.request.type = 1;
    srv.request.path_to_template = path_to_template; 
    client->call(srv); 
    file_board = srv.response.path_to_image; 
}

void task_fetch(ros::ServiceClient * client) {

    ROS_ERROR("not implemented yet");
    return; 
    // TODO TODO
    
}

void task_dialog(ros::ServiceClient * client) {


    ROS_ERROR("not implemented yet");
    return; 
    // TODO TODO
    ROS_INFO("dialog task"); 

    bwi_scavenger::Dialog srv;
    client->call(srv);
    file_dialog = srv.response.path_to_log;

}

void print_to_gui( std::vector <std::string> *tasks, 
    std::vector <Status> * task_status, ros::ServiceClient *client_list) {

    std::string message; 
    std::vector<std::string> buttons; 

    for (int i=0; i < tasks->size(); i++) {
        
        switch ((*task_status)[i]) {
            case TODO: message +=  "      "; break;
            case DOING: message += "->  "; break;
            case DONE: message +=  "done "; break;
        }

        std::string str_i = boost::lexical_cast<std::string>(i);
        buttons.push_back(str_i); 

        message += str_i + ", " + (*tasks)[i] + "\n";
    
    }

    bwi_msgs::QuestionDialog srv; 

    srv.request.type = 1;
    srv.request.message = message; 
    srv.request.options = buttons; 
    
    if (!client_list->call(srv)) 
        ROS_ERROR("Failed to call service question_dialog"); 
    
    std::string eog = "eog ";
    std::string gedit = "gedit "; 
    
    if (srv.response.index == 0 && (*task_status)[0] == DONE)
        system((eog + file_shirt).c_str());
    
    else if (srv.response.index == 1 && (*task_status)[1] == DONE)
        system((eog + file_object).c_str());
    
    else if (srv.response.index == 2 && (*task_status)[2] == DONE)
        system((eog + file_board).c_str());

    else if (srv.response.index == 3 && (*task_status)[3] == DONE)
        system((gedit + file_fetch).c_str()); 

    else if (srv.response.index == 4 && (*task_status)[4] == DONE)
        system((gedit + file_dialog).c_str()); 
}


int main(int argc, char **argv){


    ros::init(argc, argv, "scav_hunt");
    ros::NodeHandle nh;

    ros::ServiceClient client_list = nh.serviceClient <bwi_msgs::QuestionDialog> 
        ("question_dialog");

    ros::Rate rate(10); 
    ros::Duration(1.0).sleep();

    ros::param::param <std::string> ("~shirt_color", shirt_color, "blue"); 
    ROS_INFO("shirt_color: %s", shirt_color.c_str()); 

    ros::param::param <std::string> ("~object_name", object_name, "unspecified"); 
    ROS_INFO("object_name: %s", object_name.c_str()); 

    // a list of strings describing the tasks
    std::vector <std::string> tasks; 
    tasks.push_back("capture a person wearing color shirt: " + shirt_color);
    tasks.push_back("find and take a picture of object: " + object_name);
    tasks.push_back("find a person standing near a whiteboard");
    tasks.push_back("fetch an object for a person"); 
    tasks.push_back("communicate with a person in natural language"); 
    
    // status list representing TODO/DOING/DONE of each task
    std::vector <Status> task_status(tasks.size(), TODO); 

    // the buttons are always there: clicking a "DONE" button shows the result
    // clicking a "DOING"/"TODO" button does nothing

    while (ros::ok())
    {

        rate.sleep();       
        ros::spinOnce();

        std::vector<int> todo_tasks, doing_tasks, done_tasks;

        for (int i=0; i < tasks.size(); i++) {
            
            switch (task_status[i]) {
                case TODO: todo_tasks.push_back(i); break;
                case DOING: doing_tasks.push_back(i); break;
                case DONE: done_tasks.push_back(i); break;
            }
        }

        // if all tasks have been done, continue
        if (done_tasks.size() == tasks.size() || doing_tasks.size() == 1) {
            print_to_gui( & tasks, & task_status, & client_list); 
            continue;
        } 

        // if there is no "doing", but there are "todo"s
        // shuffle the todo list, and select the first one

        // std::random_shuffle (todo_tasks.begin(), todo_tasks.end()); 
        task_status[todo_tasks[0]] = DOING; 

        print_to_gui( & tasks, & task_status, & client_list); 

        ros::ServiceClient client = nh.serviceClient <bwi_scavenger::VisionTask>
            ("scavenger_vision_service"); 
        ros::ServiceClient client_dialog = nh.serviceClient 
            <bwi_scavenger::Dialog> ("dialog_service");

        switch (todo_tasks[0]) {
            // shirt object board fetch dialog
            case 0:
                task_shirt( & client); break;
            case 1:
                task_object( & client); break;
            case 2:
                task_board( & client); break;
            case 3: 
                task_fetch( & client); break;
            case 4:
                task_dialog( & client_dialog); break;
        }

        task_status[todo_tasks[0]] = DONE; 

    }
}


