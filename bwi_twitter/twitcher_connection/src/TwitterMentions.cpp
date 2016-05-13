/*
 * Copyright 2015 Ricardo Delfin Garicia <ricardo.delfin.garcia@gmail.com>
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

#include "twitcher_connection/TwitterMentions.h"

#include <iostream>

#include <ros/ros.h>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include <sstream>

TwitterMentions::TwitterMentions(int count, long since_id, 
                                 long max_id, bool trim_user,
                                 bool contributor_details,
                                 bool include_entities)
    : TwitterApiCall(), count(count), since_id(since_id),
      max_id(max_id), trim_user(trim_user),
      contributor_details(contributor_details),
      include_entities(include_entities)
{
    path = "statuses/mentions_timeline.json";
    
    std::map<std::string, std::string> queryVals;
    std::stringstream callUrl;
    
    method = HttpMethod::GET;
    
    if(count > 0)
        queryVals["count"] = std::to_string(count);
    if(since_id > 0) {
        queryVals["since_id"] = std::to_string(since_id);
    }
    if(max_id > 0)
        queryVals["max_id"] = std::to_string(max_id);
    queryVals["trim_user"] = (trim_user ? "true" : "false");
    queryVals["contributor_details"] = (contributor_details ? "true" : "false");
    queryVals["include_entities"] = (include_entities ? "true" : "false");
    
    callUrl << url << path;
    
    identity = OauthIdentity(callUrl.str(), queryVals, "GET");
    
    req = nullptr;
    
    params = std::map<std::string,std::string>(queryVals.begin(), queryVals.end());
}

const curlpp::Easy& TwitterMentions::request()
{
    if(req != nullptr)
        delete req;
    req = new curlpp::Easy;
    
    std::stringstream fullUrl;
    std::list<std::string> headers;
    
    fullUrl << url << path << (params.size() == 0 ? "" : "?");
    
    int i = 0;
    for(std::map<std::string, std::string>::iterator it = params.begin();
        it != params.end(); ++it) {
        
        fullUrl << it->first << "=" << curlpp::escape(it->second);
        if(i < params.size() - 1)
            fullUrl << "&";
        i++;
    }
    
    headers.push_back(identity.getAuthHeader());
    
    req->setOpt(new curlpp::Options::HttpHeader(headers));
    req->setOpt(new curlpp::Options::Url(fullUrl.str()));
    
    return *req;
}

TwitterMentions::~TwitterMentions()
{
     if(req != nullptr) delete req;
}
