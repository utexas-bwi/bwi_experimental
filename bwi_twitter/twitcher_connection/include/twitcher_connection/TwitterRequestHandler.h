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

#ifndef TWITTERREQUESTHANDLER_H
#define TWITTERREQUESTHANDLER_H

#include <json/json.hpp>

#include <string>
#include <map>

#include <ros/ros.h>

#include "twitcher_connection/TwitterApiCall.h"

class TwitterRequestHandler
{
public:
    TwitterRequestHandler();
    TwitterRequestHandler(const TwitterRequestHandler& other);
    ~TwitterRequestHandler();
    
    /*
     * Returns a string with the response from the API call made to the Twitter
     * REST API. apiPath should contain the path to the API call (after the
     * version number in the API URL) as well as any query parameters.
     * 
     * Example: If you want to get the first 5 retweets for the tweet with ID
     * 359 (in other words, make a request to the statuses/retweets/:id API),
     * then you would call makeRequest("statuses/retweets/id.json?count=5")
     */ 
    std::string makeRequest(TwitterApiCall*);
    
private:
    std::string authorizationHeader;
    std::string apiUrl;
    
    ros::NodeHandle nh;
};

#endif // TWITTERREQUESTHANDLER_H
