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

#ifndef TWITTERMENTIONSMONITOR_H
#define TWITTERMENTIONSMONITOR_H

#include <ros/ros.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include "twitcher_connection/TwitterRequestHandler.h"


class TwitterMentionsMonitor
{
public:
    TwitterMentionsMonitor(ros::NodeHandle&, TwitterRequestHandler handler);
    
    void timerCallback(const ros::TimerEvent);
    
    ~TwitterMentionsMonitor();
    
private:
    ros::Timer timer;
    long lastTweetId;
    
    TwitterRequestHandler handler;
    
    ros::Publisher mention_publisher;
    
    void receiveNewMentions();
    
    
    bool IsDateAfterStart(std::string date);
    std::string ToIsoString (std::string date);
    
    boost::posix_time::ptime start;
};

#endif // TWITTERMENTIONSMONITOR_H
