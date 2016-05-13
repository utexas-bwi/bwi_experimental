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
#include <string>

#include "twitcher_connection/TwitterRequestHandler.h"
#include "twitcher_connection/TwitterApiCall.h"
#include "twitcher_connection/TwitterMentions.h"
#include "twitcher_connection/TwitterUpdateStatus.h"
#include "twitcher_connection/SendTweetServer.h"
#include "twitcher_connection/TwitterMentionsMonitor.h"
#include "twitcher_connection/TwitterShowUser.h"

#include <twitcher_connection/SendTweetAction.h>
#include <twitcher_connection/handle_from_id.h>

#include <actionlib/client/simple_action_client.h>

#include <json/json.hpp>

using json = nlohmann::json;

ros::ServiceServer idConverter;
TwitterRequestHandler* handler;

bool idConverterCallback(twitcher_connection::handle_from_id::Request&, twitcher_connection::handle_from_id::Response&);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_connection_node");
    
    ros::NodeHandle nh;
    
    handler = new TwitterRequestHandler;
    
    TwitterMentionsMonitor monitor(nh, *handler);
    
    /* Initialize a SendTweet Action server */
    SendTweetServer sendTweet("send_tweet", *handler);
    
    idConverter = nh.advertiseService("twitter/handle_from_id", idConverterCallback);
    
    ros::spin();
    
    delete handler;

    return 0;
    
}

bool idConverterCallback(twitcher_connection::handle_from_id::Request& req, twitcher_connection::handle_from_id::Response& res)
{
    TwitterShowUser* showApi = new TwitterShowUser(req.id, "");
    std::string result = handler->makeRequest(showApi);
    delete showApi;
    
    if(result == "" || result == "[]") {
        ROS_WARN("Response from Show user API was empty!");
        res.handle = "";
        return false;
    }
    
    json root = json::parse(result);
    
    ROS_INFO_STREAM("Screen name result: " << root["screen_name"]);
    ROS_INFO_STREAM("Screen name lookup response: " << result);
    
    res.handle = root["screen_name"];
    
    return true;
}

