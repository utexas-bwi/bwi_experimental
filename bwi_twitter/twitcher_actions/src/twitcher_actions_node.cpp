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

#include <ros/ros.h>
#include <twitcher_actions/GoToLocationAction.h>

#include "twitcher_actions/GoToLocation.h"
#include "twitcher_actions/facedoor.h"
#include "twitcher_actions/Say.h"

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "twitcher_actions");
    ros::NodeHandle n;
    
    GoToLocation goToLocation(n);
    FaceDoor faceDoor(n);
    Say say(n);
    
    ros::spin();
}