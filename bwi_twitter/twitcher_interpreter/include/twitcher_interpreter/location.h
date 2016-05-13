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

#ifndef LOCATION_H
#define LOCATION_H

#include <ros/ros.h>
#include <XmlRpc.h>

#include <string>
#include <vector>

class Location
{
private:
  std::string asp_name;
  std::vector<std::string> common_name;  
  std::vector<std::string> doors;
public:
  Location();
  Location(const std::string &name, const std::vector<std::string> &common_name);
  
  explicit Location(std::string json);
  explicit Location(XmlRpc::XmlRpcValue val);
  
  bool isMentioned(std::string tweet);
  bool hasDoor();
  std::string getFirstDoor();
  
  std::string getAspName();
  
  std::string serialize();

  
  ~Location();
};

#endif // LOCATION_H
