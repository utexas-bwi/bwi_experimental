/*
 * Copyright 2016 Ricardo Delfin Garcia <ricardo.delfin.garcia@gmail.com>
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

#include "twitcher_connection/TwitterShowUser.h"

TwitterShowUser::TwitterShowUser(std::string userId, std::string screenName, bool entities)
    : TwitterApiCall(), userId(userId), screenName(screenName), entities(entities)
{
    path = "users/show.json";
    
    std::map<std::string, std::string> queryVals;
    std::stringstream callUrl;
    
    method = HttpMethod::GET;
    
    if(userId != "")
        queryVals["user_id"] = userId;
    if(screenName != "") {
        queryVals["screen_name"] = screenName;
    }
    if(entities)
        queryVals["include_entities"] = "true";
    
    callUrl << url << path;
    
    identity = OauthIdentity(callUrl.str(), queryVals, "GET");
    
    req = nullptr;
    
    params = std::map<std::string,std::string>(queryVals.begin(), queryVals.end());
}

const curlpp::Easy& TwitterShowUser::request()
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

TwitterShowUser::~TwitterShowUser()
{
    delete req;
}
