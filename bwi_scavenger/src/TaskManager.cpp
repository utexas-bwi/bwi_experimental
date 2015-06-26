
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "bwi_msgs/QuestionDialog.h"
#include "bwi_scavenger/TargetSearch.h"
#include "bwi_scavenger/Dialog.h"
#include "bwi_scavenger/FetchObject.h"
#include "bwi_scavenger/ScavStatus.h"

#include <boost/lexical_cast.hpp>

#include <string>
#include <stdlib.h>
#include <vector>

ros::NodeHandle * nh; 

// parameters set in this node
std::string directory = "", shirt_color = "", object_name = "", 
    file_template = "";

// parameters received from individual tasks
std::string file_shirt = "", file_object = "", file_board = "", file_fetch = "",
    file_dialog = ""; 

enum Task { WHITEBOARD=0, COLORSHIRT=1, OBJECTSEARCH=2, FETCHOBJECT=3, DIALOG=4 }; 

std::vector <std::string> task_descriptions;

std::vector <int> task_statuses; 


void task_shirt()
{
    ros::ServiceClient client = nh->serviceClient 
        <bwi_scavenger::TargetSearch> ("target_search_service"); 
    bwi_scavenger::TargetSearch srv;
    srv.request.type = 3;

    if (shirt_color.compare("red") == 0)
        srv.request.color = bwi_scavenger::TargetSearchRequest::RED; 
    else if (shirt_color.compare("blue") == 0)
        srv.request.color = bwi_scavenger::TargetSearchRequest::BLUE; 
    else if (shirt_color.compare("green") == 0)
        srv.request.color = bwi_scavenger::TargetSearchRequest::GREEN; 
    else if (shirt_color.compare("yellow") == 0)
        srv.request.color = bwi_scavenger::TargetSearchRequest::YELLOW; 

    ROS_INFO("shirt_color: %s", shirt_color.c_str()); 
    ROS_INFO("%s: task_shirt waitForExistence()", 
        ros::this_node::getName().c_str()); 
    client.waitForExistence(); 
    ROS_INFO("%s: task_shirt service ready",ros::this_node::getName().c_str()); 
    client.call(srv); 

    file_shirt = srv.response.path_to_image; 
}

void task_object()
{
    ros::ServiceClient client = nh->serviceClient 
        <bwi_scavenger::TargetSearch> ("target_search_service"); 
    bwi_scavenger::TargetSearch srv;
    srv.request.type = 2;

    srv.request.path_to_template = file_template; 

    ROS_INFO("path_to_template: %s", file_template.c_str()); 

    ROS_INFO("%s: task_object waitForExistence()", 
        ros::this_node::getName().c_str()); 
    client.waitForExistence(); 
    ROS_INFO("%s: task_object service ready",ros::this_node::getName().c_str()); 
    client.waitForExistence(); 
    client.call(srv); 
    file_object = srv.response.path_to_image; 
}

void task_board()
{
    ros::ServiceClient client = nh->serviceClient 
        <bwi_scavenger::TargetSearch> ("target_search_service"); 
    bwi_scavenger::TargetSearch srv;
    srv.request.type = 1;

    ROS_INFO("%s: task_board waitForExistence()", 
        ros::this_node::getName().c_str()); 
    client.waitForExistence(); 
    ROS_INFO("%s: task_board service ready",ros::this_node::getName().c_str()); 
    client.call(srv); 
    file_board = srv.response.path_to_image; 
}

void task_fetch() {
    ros::ServiceClient client = nh->serviceClient
        <bwi_scavenger::FetchObject> ("fetch_object_service");
    bwi_scavenger::FetchObject srv;

    ROS_INFO("%s: fetch_object waitForExistence()",
        ros::this_node::getName().c_str()); 
    client.waitForExistence(); 
    client.call(srv); 

    file_fetch = srv.response.path_to_log; 
}

void task_dialog() {
    ROS_ERROR("not implemented yet");
    return; 
}

void print_to_gui( ros::ServiceClient *gui_service_client ) {

    std::string message, str_i; 
    std::vector<std::string> buttons; 
    int number_of_tasks = task_descriptions.size(); 

    for (int i=0; i < number_of_tasks; i++) {
        
        switch ( task_statuses[i] ) {
            case bwi_scavenger::ScavStatus::TODO: message +=  "\t"; break;
            case bwi_scavenger::ScavStatus::ONGOING: message += " ->\t"; break;
            case bwi_scavenger::ScavStatus::FINISHED: message +=  " done\t"; break;
        }

        str_i.clear(); 
        str_i = boost::lexical_cast<std::string>(i);
        buttons.push_back(str_i); 

        message += str_i + ", " + task_descriptions[i];

        if ( i == COLORSHIRT )
            message += shirt_color + "\n"; 
        else if ( i == OBJECTSEARCH )
            message += object_name + "\n";
        else
            message += "\n"; 
    }

    bwi_msgs::QuestionDialog srv; 

    srv.request.type = 1;
    srv.request.message = message; 
    srv.request.options = buttons; 
    
    gui_service_client->call(srv);
    
    std::string eog = "eog ";
    std::string gedit = "gedit "; 
    
    if (srv.response.index == 0 && task_statuses[0] == bwi_scavenger::ScavStatus::FINISHED)
    {    
        system((eog + file_shirt).c_str());
        ROS_INFO("%s", (eog + file_shirt).c_str()); 
    }
    else if (srv.response.index == 1 && task_statuses[1] == bwi_scavenger::ScavStatus::FINISHED)
    {
        system((eog + file_object).c_str());
        ROS_INFO("%s", (eog + file_object).c_str()); 
    }
    else if (srv.response.index == 2 && task_statuses[2] == bwi_scavenger::ScavStatus::FINISHED)
    {
        system((eog + file_board).c_str());
        ROS_INFO("%s", (eog + file_board).c_str());
    }
    else if (srv.response.index == 3 && task_statuses[3] == bwi_scavenger::ScavStatus::FINISHED)
    {
        system((gedit + file_fetch).c_str()); 
        ROS_INFO("%s", (gedit + file_fetch).c_str());
    }
    else if (srv.response.index == 4 && task_statuses[4] == bwi_scavenger::ScavStatus::FINISHED)
    { 
        system((gedit + file_dialog).c_str()); 
        ROS_INFO("%s", (gedit + file_dialog).c_str());
    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "scav_hunt");
    nh = new ros::NodeHandle();

    ros::ServiceClient gui_service_client = nh->serviceClient 
        <bwi_msgs::QuestionDialog> ("question_dialog");

    task_descriptions.push_back("find a person standing near a whiteboard"); 
    task_descriptions.push_back("find a person wearing a color shirt and take a picture: "); 
    task_descriptions.push_back("find and take a picture of object: "); 
    task_descriptions.push_back("fetch an object for a person"); 
    task_descriptions.push_back("communicate with natural language"); 

    bwi_scavenger::ScavStatus msg; 
    task_statuses = std::vector <int32_t> 
        (task_descriptions.size(), bwi_scavenger::ScavStatus::TODO); 
    msg.names.push_back("white board"); 
    msg.names.push_back("color shirt"); 
    msg.names.push_back("object search"); 
    msg.names.push_back("object fetching"); 
    msg.names.push_back("dialog"); 

    ros::Rate rate(10); 
    ros::Duration(1.0).sleep();

    ros::param::param<std::string>("~directory", directory, "/home/bwi/shiqi/");
    ros::param::param<std::string>("~shirt_color", shirt_color, "blue"); 
    ros::param::param<std::string>("~object_name", object_name, "unspecified"); 
    ros::param::param<std::string>("~path_to_template", file_template, 
        directory + "template.jpg"); 

    // the buttons are always there: clicking a "DONE" button shows the result
    // clicking a "DOING"/"TODO" button does nothing

    std::vector<int> todo_tasks, doing_tasks, done_tasks;
    int number_of_tasks = task_descriptions.size(); 

    ros::Publisher status_pub = nh->advertise <bwi_scavenger::ScavStatus> 
        ("scav_hunt_status", 100, true); 

    ROS_INFO("%s: initialization finished", ros::this_node::getName().c_str()); 
    while (ros::ok()) {

        rate.sleep();       
        ros::spinOnce();

        todo_tasks.clear(); doing_tasks.clear(); done_tasks.clear(); 

        msg.statuses.clear(); 
        for (int i=0; i < number_of_tasks; i++) {

            msg.statuses.push_back(task_statuses[i]);

            switch ( task_statuses[i] ) {
                case bwi_scavenger::ScavStatus::TODO: 
                    todo_tasks.push_back(i); break;
                case bwi_scavenger::ScavStatus::ONGOING: 
                    doing_tasks.push_back(i); break;
                case bwi_scavenger::ScavStatus::FINISHED: 
                    done_tasks.push_back(i); break;
            }
        }

        status_pub.publish(msg);


        if (done_tasks.size() == number_of_tasks) //if all tasks have been done
        {
            ROS_INFO("all tasks have been finished"); 
            return 0; 
        }
            
        if (doing_tasks.size() == 1) // if the robot is busy
        {
            ROS_INFO("robot busy"); 
            print_to_gui( & gui_service_client ); 
            ros::Duration(1.0).sleep(); 
            continue; 
        }

        ROS_INFO("%s: do the 1st on todo list", ros::this_node::getName().c_str()); 
        // otherwise, change the status first todo task into "DOING"
        task_statuses[todo_tasks[0]] = bwi_scavenger::ScavStatus::ONGOING; 
        print_to_gui( & gui_service_client); 

        ROS_INFO("%s: select a task", ros::this_node::getName().c_str()); 
        switch (static_cast <Task> (todo_tasks[0]) ) {

            case WHITEBOARD:
                task_board(); break;
            case COLORSHIRT:
                task_shirt(); break;
            case OBJECTSEARCH:
                task_object(); break;
            case FETCHOBJECT: 
                task_fetch(); break;
            case DIALOG:
                task_dialog(); break;
        }
        ROS_INFO("%s: task done", ros::this_node::getName().c_str()); 

        task_statuses[todo_tasks[0]] = bwi_scavenger::ScavStatus::FINISHED; 

    }
}


