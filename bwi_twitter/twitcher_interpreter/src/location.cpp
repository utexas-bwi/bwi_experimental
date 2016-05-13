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

#include <twitcher_interpreter/location.h>
#include <json/json.hpp>
#include <string>

#include <XmlRpc.h>

#include <ros/ros.h>

#include <string>

using json = nlohmann::json;

Location::Location()
{

}

Location::Location(const std::string &name, const std::vector<std::string> &common_name)
  : asp_name(name), common_name(common_name)
{
    
}

Location::Location(std::string json_str)
{
    json root = json::parse(json_str);
    asp_name = root["asp_name"].get<std::string>();
    
    for(int i = 0; i < root["common_names"].size(); i++) {
	common_name.push_back(root["common_names"][i].get<std::string>());
    }
    
    for(int i = 0; i < root["doors"].size(); i++) {
        doors.push_back(root["doors"][i].get<std::string>());
    }
}

Location::Location(XmlRpc::XmlRpcValue val) {
    asp_name = static_cast<std::string>(val["asp_name"]);
    
    for(int i = 0; i < val["common_names"].size(); i++) {
        std::string name = val["common_names"][i];
        common_name.push_back(name);
    }
    
    if(val.hasMember("doors")) {
        for(int i = 0; i < val["doors"].size(); i++) {
            std::string door = val["doors"][i];
            doors.push_back(door);
        }
    }
}

bool Location::isMentioned(std::string tweet)
{
  for(auto it = common_name.begin(); it != common_name.end(); ++it)
    if(tweet.find(*it) != std::string::npos)
	return true;
  
  return false;
}

// True if the room has a door. False otherwise
bool Location::hasDoor()
{
    return doors.size() != 0;
}

// Return the empty string if there are no doors. Otherwise, return the first door in the list
std::string Location::getFirstDoor()
{
    return (doors.size() == 0 ? "" : doors[0]);
}

std::string Location::getAspName() {
    return this->asp_name;
}

std::string Location::serialize() {
    json j;
    j["asp_name"] = asp_name;
    j["common_name"] = common_name;
    j["doors"] = doors;
    
    return j.dump();
}

Location::~Location()
{
    
}
