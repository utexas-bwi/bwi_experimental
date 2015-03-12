
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "segbot_gui/QuestionDialog.h"

#include <string>
#include <stdlib.h>
#include <vector>

std::string status_shirt, status_object, status_board;
std::string shirt_color, object_name;
std::string file_shirt, file_object, file_board; 


void callback_shirt(const std_msgs::String::ConstPtr& msg)
{
 
    if (msg->data.find("running") != std::string::npos) 
        status_shirt = "ongoing";
    else {
        status_shirt = "finished";
        int pos = msg->data.find(":"); 
        file_shirt = msg->data.substr(pos + 1); 
    }
        
}

void callback_object(const std_msgs::String::ConstPtr& msg)
{
 
    if (msg->data.find("running") != std::string::npos)       
        status_object = "ongoing";
    else {
        status_object = "finished";
        int pos = msg->data.find(":");
        file_object = msg->data.substr(pos + 1);
    }

}

void callback_board(const std_msgs::String::ConstPtr& msg)
{
 
    if (msg->data.find("running") != std::string::npos)       
        status_board = "ongoing";
    else {
        status_board = "finished";
        int pos = msg->data.find(":");
        file_board = msg->data.substr(pos + 1);
    }
        
}

int main(int argc, char **argv){

    ros::init(argc, argv, "scav_hunt");
    ros::NodeHandle nh;

    ros::ServiceClient client = 
            nh.serviceClient<segbot_gui::QuestionDialog>("question_dialog");
    segbot_gui::QuestionDialog srv; 

    ros::Subscriber sub1 = nh.subscribe("segbot_blue_shirt_status", 1000,
                                        callback_shirt);

    ros::Subscriber sub2 = nh.subscribe("segbot_object_detection_status", 1000,
                                        callback_object);
 
    ros::Subscriber sub3 = nh.subscribe("segbot_whiteboard_status", 1000,
                                        callback_board);
        
    ros::Rate rate(10); 

    std::string message_finished, message_todo, message_ending; 
    std::vector<std::string> buttons; 

    message_finished = "Finished tasks:";
    message_todo = "\nTodo tasks:";
    message_ending = "\nClick buttons to see how I performed in the tasks\n";

    while (ros::ok())
    {
        
        rate.sleep();       
        ros::spinOnce();

        if (status_shirt.find("ongoing") != std::string::npos) 
            message_todo += "\n\t* find a person wearing '" +
                            shirt_color + " shirt"; 
        else 
        {
            message_finished += "\n\t* find a person wearing '" + 
                                shirt_color + " shirt"; 
            buttons.push_back("color shirt");
        }
 
            
        if (status_object.find("ongoing") != std::string::npos) 
            message_todo += "\n\t* take a picture of object '" + object_name; 
        else 
        {
            message_finished += "\n\t* take a picture of object '" + object_name;
            buttons.push_back("object detection");
        }


        if (status_board.find("ongoing") != std::string::npos) 
            message_todo += "\n\t* find a person standing near a whiteboard"; 
        else 
        {
            message_finished += "\n\t* find a person standing near a whiteboard"; 
            buttons.push_back("whiteboard");
        }


        if (message_finished.length() + message_todo.length() < 30) {
            
            srv.request.type = 0;
            srv.request.message = message_finished + "\n" + message_todo + 
                                  "\n" + message_ending; 
        }
        else 
        {
            srv.request.type = 1;
            srv.request.message = message_finished + "\n" + message_todo; 
            srv.request.options = buttons; 
        }

        if (!client.call(srv)) 
        {
            ROS_ERROR("Failed to call service question_dialog"); 
            return 1;
        }

        std::string eog = "eog ";

        if (srv.response.index < 0) 
            continue;

        else if (buttons[srv.response.index].find("shirt") != std::string::npos)
            system((eog + file_shirt).c_str());

        else if (buttons[srv.response.index].find("object") != std::string::npos)
            system((eog + file_object).c_str());

        else if (buttons[srv.response.index].find("board") != std::string::npos)
            system((eog + file_board).c_str());

    }
}





