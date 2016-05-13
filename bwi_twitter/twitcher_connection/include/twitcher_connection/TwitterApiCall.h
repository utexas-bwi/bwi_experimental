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

#ifndef __TWITCHER_CONNECTION_TWITTERAPICALL_H__
#define __TWITCHER_CONNECTION_TWITTERAPICALL_H__

#include "twitcher_connection/OauthIdentity.h"

#include <string>
#include <map>

#include <ros/ros.h>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Easy.hpp>

/* Enumeration of HTTP methods used by the twitter API. Can be expanded as
 * necesary
 */
enum class HttpMethod {
    GET,
    POST,
    PATCH,
    PUT
};

class TwitterApiCall {
public:
    TwitterApiCall();
    
    virtual const curlpp::Easy& request() = 0;
    
    virtual ~TwitterApiCall() { }
    
protected:
    std::string url;
    std::string path;
    std::map<std::string, std::string> params;
    std::string body;
    HttpMethod method;
    OauthIdentity identity;
    curlpp::Easy *req;
    
    ros::NodeHandle nh;
};

#endif