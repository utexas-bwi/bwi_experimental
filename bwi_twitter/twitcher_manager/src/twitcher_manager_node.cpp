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

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <json/json.hpp>

#include "twitcher_connection/Tweet.h"
#include <twitcher_connection/handle_from_id.h>
#include <twitcher_connection/SendTweetAction.h>

#include <twitcher_interpreter/interpret_dialog.h>

#include <twitcher_actions/FaceDoorAction.h>
#include <twitcher_actions/GoToLocationAction.h>
#include <twitcher_actions/SayAction.h>

#include <std_srvs/Empty.h>

using json = nlohmann::json;

ros::ServiceClient interpreterClient;


actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>* goToLocationClient;
actionlib::SimpleActionClient<twitcher_actions::FaceDoorAction>* faceDoorClient;
actionlib::SimpleActionClient<twitcher_actions::SayAction>* sayClient;

actionlib::SimpleActionClient<twitcher_connection::SendTweetAction>* sendTweetClient;

ros::ServiceClient betweenDoorsResumeClient;

bool useApi, demoMode;

ros::ServiceClient handleFromIdClient;

void tweetReceived(const twitcher_connection::Tweet::ConstPtr&);
void actOnTweet(const twitcher_connection::Tweet::ConstPtr&, const twitcher_interpreter::interpret_dialog::Response&);
void goToLocationAndSay(const std::string&, const std::string&);
void goToLocation(const std::string&);
void sendResponse(const std::string&, const std::string&);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_manager_node");
    
    ros::NodeHandle node;
    
    // Parameters to load in:
    if(!node.getParam("/twitter/useapi", useApi))
        useApi = true;
    
    if(!node.getParam("/twitter/demomode", demoMode))
        demoMode = false;
    
    // Connect to interpreter
    interpreterClient = node.serviceClient<twitcher_interpreter::interpret_dialog>("twitter/interpret_message");
    interpreterClient.waitForExistence();
    
    // Connect to twitcher_connection handle from id
    if(useApi) {
        handleFromIdClient = node.serviceClient<twitcher_connection::handle_from_id>("twitter/handle_from_id");
        handleFromIdClient.waitForExistence();
    }
    
    // Connect to twitter_mentions (from twitcher_connection)
    ros::Subscriber subscriber = node.subscribe("twitter_mentions", 1000, tweetReceived);
    
    // Set up action clients for twitcher_actions
    goToLocationClient = new actionlib::SimpleActionClient<twitcher_actions::GoToLocationAction>(node, "GoToLocation", true);
    faceDoorClient = new actionlib::SimpleActionClient<twitcher_actions::FaceDoorAction>(node, "FaceDoor", true);
    sayClient = new actionlib::SimpleActionClient<twitcher_actions::SayAction>(node, "SayTwitter", true);
    
    if(useApi) {
        sendTweetClient = new actionlib::SimpleActionClient<twitcher_connection::SendTweetAction>(node, "send_tweet", true);
        sendTweetClient->waitForServer();
    }
    
    if(demoMode) {
        betweenDoorsResumeClient = node.serviceClient<std_srvs::Empty>("/between_doors_interruptible/resume");
    }

    goToLocationClient->waitForServer();
    faceDoorClient->waitForServer();
    sayClient->waitForServer();
    
    ros::spin();
    
    delete goToLocationClient;
    delete faceDoorClient;
    delete sayClient;
    
    if(sendTweetClient)
        delete sendTweetClient;
}


void tweetReceived(const twitcher_connection::Tweet::ConstPtr& tweet)
{
    ROS_INFO_STREAM("Manager received tweet from /twitter_mentions. Content: \""
                    << tweet->message << "\". Forwarding to /dialog");
    
    twitcher_interpreter::interpret_dialog::Request req;
    twitcher_interpreter::interpret_dialog::Response res;
    
    req.message = tweet->message;
    req.user_id = tweet->sender;
    req.datetime = tweet->sentTime;
    
    interpreterClient.call(req, res);
    
    actOnTweet(tweet, res);
}

void actOnTweet(const twitcher_connection::Tweet::ConstPtr& tweet, const twitcher_interpreter::interpret_dialog::Response& res) {
    switch(res.action) {
        case twitcher_interpreter::interpret_dialog::Response::GO_TO_ACTION:
            sendResponse("Alrighty! I'm on my way.", tweet->sender);
            goToLocation(res.string_args[0]);
            sendResponse("I just got there!", tweet->sender);
            break;
        case twitcher_interpreter::interpret_dialog::Response::GO_TO_AND_SAY:
            sendResponse("Alrighty! I'll be sure to say that.", tweet->sender);
            goToLocationAndSay(res.string_args[0], res.string_args[1]);
            sendResponse("The appropriate person has been annoyed!", tweet->sender);
            break;
    }
    
    // Resume between_doors_interruptible if we are in demo mode
    if(demoMode) {
        ROS_INFO("Resuming between doors task...");

        std_srvs::EmptyRequest req;
        std_srvs::EmptyResponse res;
        betweenDoorsResumeClient.call(req, res);
    }
}


void goToLocationAndSay(const std::string& location, const std::string& text)
{
    goToLocation(location);
    
    ROS_INFO_STREAM("Starting to speak... Saying \"" << text << "\"");
    
    twitcher_actions::SayGoal say_goal;
    say_goal.message = text;
    sayClient->sendGoal(say_goal);
    sayClient->waitForResult();
    
    ROS_INFO("Say finished!");
}

void goToLocation(const std::string& location)
{
    json roomData = json::parse(location);
    
    ROS_INFO_STREAM("Ready for location!");
    ROS_INFO_STREAM("Going to location " << roomData["asp_name"]);
    
    if(roomData["doors"].size() != 0) {
        ROS_INFO("Sending face door!");
        
        twitcher_actions::FaceDoorGoal face_goal;
        face_goal.door_name = roomData["doors"][0];
        faceDoorClient->sendGoal(face_goal);
        faceDoorClient->waitForResult();
        
        ROS_INFO("Finished face door action!");
    }
    else {
        ROS_INFO("Sending go to goal! Setup...");
        
        twitcher_actions::GoToLocationGoal goto_goal;
        goto_goal.location_name = roomData["asp_name"];
        goToLocationClient->sendGoal(goto_goal);
        goToLocationClient->waitForResult();
        
        ROS_INFO("Finished go to action!");
    }
}

void sendResponse(const std::string& message, const std::string& user_id) {
    if(!useApi)
        return;
    
    ROS_INFO_STREAM("Sending tweet: \"" << message << "\" to user ID " << user_id);
    
    twitcher_connection::handle_from_id::Request req;
    twitcher_connection::handle_from_id::Response res;
    
    req.id = user_id;
    res.handle = user_id;
    
    handleFromIdClient.call(req, res);
    
    twitcher_connection::SendTweetGoal goal;
    goal.message = "@" + res.handle + " " + message;
    
    ROS_INFO_STREAM("Waiting for send result...");
    sendTweetClient->sendGoal(goal);
    sendTweetClient->waitForResult();
    ROS_INFO_STREAM("Tweet send finished!");
}
