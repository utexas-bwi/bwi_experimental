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

#include "json/json.hpp"
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/thread/pthread/mutex.hpp>

#include "twitcher_connection/TwitterMentionsMonitor.h"
#include "twitcher_connection/TwitterMentions.h"

#include "twitcher_connection/Tweet.h"

using json = nlohmann::json;

TwitterMentionsMonitor::TwitterMentionsMonitor(ros::NodeHandle& nh, 
                                               TwitterRequestHandler handler) : 
                                               lastTweetId(-1), handler(handler)
{
    ROS_INFO("Hello from metionsmonitors!");
    /* Call the timer callback every minute. This will cause the program to
     * process all the tweets and send them as messages through a topic */
    timer = nh.createTimer(ros::Duration(60.0), boost::bind(&TwitterMentionsMonitor::timerCallback, this, _1));
    mention_publisher = nh.advertise<twitcher_connection::Tweet>("twitter_mentions", 1000);
    start = boost::posix_time::second_clock::universal_time();
    receiveNewMentions();
}
    
void TwitterMentionsMonitor::timerCallback(const ros::TimerEvent te)
{
    receiveNewMentions();
}

void TwitterMentionsMonitor::receiveNewMentions()
{
    ROS_INFO_STREAM("Receiving new mentions... Start (UTC): "
                    << boost::posix_time::to_iso_string(start));
    
    TwitterApiCall* call = new TwitterMentions(-1, lastTweetId, -1, false, false, false);
    
    std::string result = handler.makeRequest(call);
    
    ROS_INFO("We received the following mentions:");
    ROS_INFO("%s", result.c_str());
    
    if(result == "" || result == "[]") return;
    
    json root = json::parse(result);
    
    // Set the last tweet ID to the 0th index in the array (always the latest)
    lastTweetId = root[0]["id"];
    
    // Iterate in reverse temporal order
    for(int i = root.size() - 1; i >= 0; i--) {
        ROS_INFO("Processing tweet. Information:");
        ROS_INFO_STREAM("\tMessage: " << root[(int)i]["text"]);
        ROS_INFO_STREAM("\tSent Time (ISO): " << ToIsoString(root[(int)i]["created_at"]));
        
        if(IsDateAfterStart(root[(int)i]["created_at"])) {
            ROS_INFO("Accepted! Forwarding...");
            twitcher_connection::Tweet tweet;
            tweet.id = root[(int)i]["id"];
            tweet.message = root[(int)i]["text"];
            tweet.sender = root[(int)i]["user"]["id_str"];
            tweet.sentTime = ToIsoString(root[(int)i]["created_at"]);
            
            mention_publisher.publish<twitcher_connection::Tweet>(tweet);
            
        }
    }
    
    delete call;
}

bool TwitterMentionsMonitor::IsDateAfterStart(std::string date)
{
    std::stringstream dateStream;   
    dateStream.exceptions(std::ios_base::failbit);
    
    boost::posix_time::time_input_facet *facet = new boost::posix_time::time_input_facet("%a %b %d %H:%M:%S +0000 %Y"); // "Mon Sep 03 13:24:14 +0000 2012"
    boost::posix_time::ptime time;
    
    // Get the current time in UTC for comparison
    
    // Setup date stream to output using facet
    dateStream.str(date); 
    dateStream.imbue(std::locale(dateStream.getloc(), facet));
    
    // Convert
    dateStream >> time;
    
    //delete facet;
    
    // Compare to the time the server started (NOT the current time)
    return start < time;
}

std::string TwitterMentionsMonitor::ToIsoString(std::string date)
{
    std::stringstream dateStream;
    // Throw expception if date is formatted differently
    dateStream.exceptions(std::ios_base::failbit);
    
    // Format for the input Twitter date
    boost::posix_time::time_input_facet *facet = new boost::posix_time::time_input_facet("%a %b %d %H:%M:%S +0000 %Y"); // "Mon Sep 03 13:24:14 +0000 2012"
    boost::posix_time::ptime time;
    
    // Setup date stream to output using facet
    dateStream.str(date); 
    dateStream.imbue(std::locale(dateStream.getloc(), facet));
    
    // Convert
    dateStream >> time;
    
    //delete facet;
    
    return boost::posix_time::to_iso_string(time);
}


TwitterMentionsMonitor::~TwitterMentionsMonitor()
{
    
}
