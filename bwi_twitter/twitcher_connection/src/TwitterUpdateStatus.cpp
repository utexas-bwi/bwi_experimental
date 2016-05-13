/*
 * Copyright 2015 <copyright holder> <email>
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

#include "twitcher_connection/TwitterUpdateStatus.h"

#include <sstream>

TwitterUpdateStatus::TwitterUpdateStatus(std::string status,
                                         int in_reply_to_status_id,
                                         bool possibly_sensitive,
                                         bool trim_user)

    : TwitterApiCall(), status(status),
      in_reply_to_status_id(in_reply_to_status_id),
        possibly_sensitive(possibly_sensitive), trim_user(trim_user)
{
    path = "statuses/update.json";
    
    std::map<std::string, std::string> queryVals;
    std::stringstream callUrl;
    
    method = HttpMethod::POST;
    
    queryVals["status"] = status;
    if(in_reply_to_status_id > 0)
        queryVals["in_reply_to_status_id"] = std::to_string(in_reply_to_status_id);
    
    queryVals["trim_user"] = (trim_user ? "true" : "false");
    queryVals["possibly_sensitive"] = (possibly_sensitive ? "true" : "false");
    
    callUrl << url << path;
    
    identity = OauthIdentity(callUrl.str(), queryVals, "POST");
    
    req = nullptr;
    
    params = std::map<std::string,std::string>(queryVals.begin(), queryVals.end());
}


const curlpp::Easy& TwitterUpdateStatus::request()
{
    if(req != nullptr)
        delete req;
    req = new curlpp::Easy;
    
    std::stringstream fullUrl;
    std::stringstream paramStream;
    std::list<std::string> headers;
    
    fullUrl << url << path;
    
    int i = 0;
    for(std::map<std::string, std::string>::iterator it = params.begin();
        it != params.end(); ++it) {
        
        paramStream << it->first << "=" << curlpp::escape(it->second);
        if(i < params.size() - 1)
            paramStream << "&";
        i++;
    }
    
    headers.push_back(identity.getAuthHeader());
    headers.push_back("Content-Type: application/x-www-form-urlencoded");
    
    req->setOpt(new curlpp::Options::HttpHeader(headers));
    req->setOpt(new curlpp::Options::Url(fullUrl.str()));
    req->setOpt(new curlpp::Options::Post(true));
    req->setOpt(new curlpp::Options::PostFields(paramStream.str()));
    req->setOpt(new curlpp::Options::PostFieldSize(paramStream.str().length()));
    
    return *req;
}

TwitterUpdateStatus::~TwitterUpdateStatus()
{

}

