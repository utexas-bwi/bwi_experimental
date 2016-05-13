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

#ifndef TWITTERUPDATESTATUS_H
#define TWITTERUPDATESTATUS_H

#include "twitcher_connection/TwitterApiCall.h"

#include <string>

class TwitterUpdateStatus : public TwitterApiCall
{
public:
    TwitterUpdateStatus(std::string status, int in_reply_to_status_id, 
                        bool possibly_sensitive, bool trim_user);
    
    virtual const curlpp::Easy& request();
    
    ~TwitterUpdateStatus();
    
private:
    std::string status;
    int in_reply_to_status_id;
    bool possibly_sensitive;
    bool trim_user;
    
    
};

#endif // TWITTERUPDATESTATUS_H
