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

#include "twitcher_connection/OauthIdentity.h"
#include "twitcher_connection/base64.h"
#include "twitcher_connection/HMAC_SHA1.h"

#include "json/json.hpp"

#include <curlpp/Easy.hpp>
#include <curlpp/cURLpp.hpp>

#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <ctime>
#include <cstring>

#include <boost/config/posix_features.hpp>

using json = nlohmann::json;

OauthIdentity::OauthIdentity(std::string consumerKey, std::string consumerSecret,
                  std::string accessToken, std::string accessTokenSecret, 
                  std::string apiUrl, std::string version,
                  std::map<std::string, std::string> queryVals,
                  std::string httpMethod)
                  : consumerKey(consumerKey), consumerSecret(consumerSecret),
                    accessToken(accessToken), accessTokenSecret(accessTokenSecret),
                    version(version), apiUrl(apiUrl), queryVals(queryVals),
                    httpMethod(httpMethod)
{
    srand(time(NULL));
}

OauthIdentity::OauthIdentity(std::string apiUrl, 
                             std::map<std::string, std::string> queryVals,
                             std::string httpMethod)
                            : queryVals(queryVals), httpMethod(httpMethod), apiUrl(apiUrl)
                            
{
    srand(time(NULL));
    
    if(!nh.getParam("/twitter/oauth/consumer_key", consumerKey))
        ROS_ERROR("OauthIdentity could not load /twitter/oauth/consumer_key! Check config/config.yaml in the twitcher_launch package.");
    
    if(!nh.getParam("/twitter/oauth/consumer_secret", consumerSecret))
        ROS_ERROR("TwitterApiCall could not load /twitter/oauth/consumer_secret! Check config/config.yaml in the twitcher_launch package.");
    
    if(!nh.getParam("/twitter/oauth/token", accessToken))
        ROS_ERROR("TwitterApiCall could not load /twitter/oauth/token! Check config/config.yaml in the twitcher_launch package.");
    
    if(!nh.getParam("/twitter/oauth/token_secret", accessTokenSecret))
        ROS_ERROR("TwitterApiCall could not load /twitter/oauth/token_secret! Check config/config.yaml in the twitcher_launch package.");
    
    if(!nh.getParam("/twitter/oauth/version", version))
        ROS_ERROR("TwitterApiCall could not load /twitter/oauth/version! Check config/config.yaml in the twitcher_launch package.");
}

const std::string& OauthIdentity::getAuthHeader()
{
    std::stringstream headerStream;
    
    signRequest();
    
    headerStream << "Authorization: OAuth "
                    "oauth_consumer_key=\"" << consumerKey << 
                    "\", oauth_nonce=\"" << nonce <<
                    "\", oauth_signature=\"" << curlpp::escape(signature) <<
                    "\", oauth_signature_method=\"HMAC-SHA1\", "
                    "oauth_timestamp=\"" << timestamp <<
                    "\", oauth_token=\"" << accessToken <<
                    "\", oauth_version=\"" << version << "\"";
    
    authHeader = headerStream.str();
    
    return authHeader;
}


static char charMap[] = {
                         '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                         'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
                         'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
                         'w', 'x', 'y', 'z',
                         'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K',
                         'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V',
                         'W', 'X', 'Y', 'Z'
                        };

void OauthIdentity::generateOauthNonce()
{
    char c;
    std::stringstream resultStream;
    
    // Generates 42 random characters and appends them to the resultStream
    for(int i = 0; i < 42; i++)
    {
        c = charMap[rand() % 62];
        resultStream << c;
    }
    
    nonce = resultStream.str();
}

void OauthIdentity::signRequest()
{
    std::map<std::string, std::string> values;
    std::stringstream parametersStream;
    std::string paramString;
    std::stringstream dataStream;
    std::string dataString;
    std::string signingKey;
    std::stringstream signingKeyStream;
    
    generateOauthNonce();
    this->timestamp = std::string(std::to_string(time(NULL)));
    
    for(std::map<std::string, std::string>::iterator it = queryVals.begin();
        it != queryVals.end(); ++it)
        values[it->first] = curlpp::escape(it->second);
    
    values["oauth_consumer_key"] = consumerKey;
    values["oauth_nonce"] = nonce;
    values["oauth_signature_method"] = "HMAC-SHA1";
    values["oauth_timestamp"] = timestamp;
    values["oauth_token"] = accessToken;
    values["oauth_version"] = version;
    
    
    for(std::map<std::string, std::string>::iterator it = values.begin();
            it != values.end(); ++it) {
        parametersStream << it->first << "=" << it->second << "&"; 
    }
    
    paramString = parametersStream.str();
    paramString.erase(paramString.length() - 1, 1);
    
    dataStream << httpMethod << "&" << curlpp::escape(apiUrl)
               << "&" << curlpp::escape(paramString);
    dataString = dataStream.str();
                 
    signingKeyStream << curlpp::escape(consumerSecret)
                     << "&" << curlpp::escape(accessTokenSecret);
    signingKey  = signingKeyStream.str();
    
    // Generate HMAC-SHA1 key
    CHMAC_SHA1 hmac_sha1;
    unsigned char strDigest[255];
    memset( strDigest, 0, 255);
         
    hmac_sha1.HMAC_SHA1((unsigned char*)dataString.c_str(), dataString.length(),
                        (unsigned char*)signingKey.c_str(), signingKey.length(),
                        strDigest);
    
    //                                    v--SHA1 returns 20 bytes
    this->signature = base64_encode(strDigest, 20);
}

OauthIdentity::~OauthIdentity()
{

}
