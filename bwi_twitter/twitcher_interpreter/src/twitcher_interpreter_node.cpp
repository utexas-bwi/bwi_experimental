/*
 * Copyright 2015 Ricardo Delfin Garcia <ricardo.delfin.garcia@gmail.com>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include <iostream>
#include <regex>

#include <fstream>

#include <ros/ros.h>
#include <XmlRpc.h>
#include <actionlib/client/simple_action_client.h>

#include <twitcher_actions/GoToLocationAction.h>
#include <twitcher_actions/FaceDoorAction.h>
#include <twitcher_actions/SayAction.h>

#include <twitcher_interpreter/interpret_dialog.h>

//#include "twitcher_interpreter/dialog_message.h"
#include "twitcher_interpreter/location.h"

std::vector<Location> loc;
std::regex goToAndSayTweetRegex, goToTweetRegex;

std::string tempString;


bool interpreterCallback(twitcher_interpreter::interpret_dialog::Request &req,
                         twitcher_interpreter::interpret_dialog::Response &res);

void initLocations(ros::NodeHandle&);
bool locExists(std::string msg, Location** loc);
void gotoDoorAndSay(Location* location, std::string spoken_text);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_interpreter_node");
    ros::NodeHandle node;
    
    goToTweetRegex = std::regex("Go to ([.\\w\\s]+)",
                                std::regex_constants::ECMAScript | std::regex_constants::icase);
    goToAndSayTweetRegex = std::regex("Go to ([.\\w\\s]+) and say ([\\w\\s]+)",
                                      std::regex_constants::ECMAScript | std::regex_constants::icase);
    
    initLocations(node);
    
    
    ROS_INFO_STREAM("Twitcher Interpreter Node up, listening on /GoToLocation");
    
    ros::ServiceServer server = node.advertiseService("twitter/interpret_message", interpreterCallback);
    //ros::Subscriber subscriber = node.subscribe("dialog", 100, messageReceiver);
    
    ros::spin();
}

void initLocations(ros::NodeHandle& nh) {
    ROS_INFO("Loading locations...");
    
    XmlRpc::XmlRpcValue rooms;
    
    if(nh.getParam("/twitter/rooms", rooms)) {
        for(int i = 0; i < rooms.size(); i++) {
            ROS_INFO_STREAM("LOC #" << i);
            XmlRpc::XmlRpcValue room = rooms[i];
            Location newLoc = Location(rooms[i]);
            loc.push_back(newLoc);
        }
    }
    else {
        ROS_ERROR("Rooms cannot be loaded! Check config/rooms.yaml in the twitcher_launch package.");
    }
    
    ROS_INFO("Locations loaded!");
    return;
}

bool interpreterCallback(twitcher_interpreter::interpret_dialog::Request& req, twitcher_interpreter::interpret_dialog::Response& res)
{
    ROS_INFO_STREAM("Interpreter received message: \"" << req.message << 
                    "\" with timestamp " << req.datetime);
    
    std::string msgString = req.message;
    
    auto it1 = std::sregex_iterator(msgString.begin(), msgString.end(), goToAndSayTweetRegex);
    auto it2 = std::sregex_iterator(msgString.begin(), msgString.end(), goToTweetRegex);
    
    
    // Default to not finding any relevant text
    res.action = twitcher_interpreter::interpret_dialog::Response::NONE;
    
    // The string was found. Execute action
    if(it1 != std::sregex_iterator()) {
        ROS_INFO("Tweet matched with go to location and say!");
        std::smatch match = *it1;
        std::string location_name = match.str(1);
        std::string spoken_text = match.str(2);
        Location *location;
        if(locExists(location_name, &location)) {
            ROS_INFO_STREAM("Location matched to: " << location->getAspName() << " with message \"" << spoken_text << "\"");
            res.action = twitcher_interpreter::interpret_dialog::Response::GO_TO_AND_SAY;
            res.string_args.push_back(location->serialize());
            res.string_args.push_back(spoken_text);
        }
    }
    else if(it2 != std::sregex_iterator()) {
        ROS_INFO("Tweet matched with go to location!");
        std::smatch match = *it2;
        std::string location_name = match.str(1);
        Location *location;
        if(locExists(location_name, &location)) {
            ROS_INFO_STREAM("Location matched to: " << location->getAspName());
            res.action = twitcher_interpreter::interpret_dialog::Response::GO_TO_ACTION;
            res.string_args.push_back(location->serialize());
        }
    }
    else
    {
        ROS_INFO("Tweet did not match!");
    }
    
    return true;
}

bool locExists(std::string msg, Location** result) {
    for(auto it = loc.begin(); it != loc.end(); ++it) {
        if(it->isMentioned(msg)) {
            *result = &(*it);
            return true;
        }
    }
    
    return false;
}
