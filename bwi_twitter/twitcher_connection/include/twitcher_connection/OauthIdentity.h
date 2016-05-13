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

#ifndef OAUTHIDENTITY_H
#define OAUTHIDENTITY_H

#include <string>
#include <map>

#include <ros/ros.h>

class OauthIdentity
{
public:
    OauthIdentity() { }
    OauthIdentity(std::string consumerKey, std::string consumerSecret,
                  std::string accessToken, std::string accessTokenSecret, 
                  std::string apiUrl, std::string version,
                  std::map<std::string, std::string> queryVals,
                  std::string httpMethod);
    
    OauthIdentity(std::string apiUrl,
                  std::map<std::string, std::string> queryVals,
                  std::string httpMethod);
    
    const std::string& getAuthHeader();
    
    
    ~OauthIdentity();
private:
    std::string accessTokenSecret;
    std::string accessToken;
    std::string version;
    std::string consumerSecret;
    std::string consumerKey;
    std::string apiUrl;
    std::string httpMethod;
    
    std::map<std::string, std::string> queryVals;
    
    std::string signature;
    std::string authHeader;
    std::string nonce;
    std::string timestamp;
    
    ros::NodeHandle nh;
    
    void generateOauthNonce();
    
    void signRequest();
};

#endif // OAUTHIDENTITY_H
