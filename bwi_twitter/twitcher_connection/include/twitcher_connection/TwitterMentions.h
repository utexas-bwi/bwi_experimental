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

#ifndef TWITTERMENTIONS_H
#define TWITTERMENTIONS_H

#include "twitcher_connection/TwitterApiCall.h"
#include "twitcher_connection/TwitterRequestHandler.h"

#include <curlpp/Easy.hpp>
#include <string>

class TwitterMentions : public TwitterApiCall
{
public:
    TwitterMentions(int count, long since_id, long max_id,
                    bool trim_user, bool contributor_details,
                    bool include_entities);
    
    virtual const curlpp::Easy& request();
    
    ~TwitterMentions();
    
private:
    int count;
    long since_id;
    long max_id;
    bool trim_user;
    bool contributor_details;
    bool include_entities;
    
    TwitterRequestHandler handler;
};

#endif // TWITTERMENTIONS_H
