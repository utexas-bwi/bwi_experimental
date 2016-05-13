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

#include "twitcher_connection/TwitterRequestHandler.h"
#include "twitcher_connection/OauthIdentity.h"

#include "json/json.hpp"

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>

#include <sstream>
#include <ctime>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

using json = nlohmann::json;

TwitterRequestHandler::TwitterRequestHandler()
{
    if(!nh.getParam("/twitter/twitter_api_url", apiUrl))
        ROS_ERROR("TwitterRequestHandler could not load twitter_api_url! Check config/config.yaml in the twitcher_launch package.");
}

TwitterRequestHandler::TwitterRequestHandler(const TwitterRequestHandler& other)
    : authorizationHeader(other.authorizationHeader), apiUrl(other.apiUrl)
{
    
}

std::string TwitterRequestHandler::makeRequest(TwitterApiCall* call)
{
    curlpp::Cleanup cleanup;
    
    const curlpp::Easy& request = call->request();
    std::stringstream result;
    
    result << request;
    
    return result.str();
}


TwitterRequestHandler::~TwitterRequestHandler()
{

}
