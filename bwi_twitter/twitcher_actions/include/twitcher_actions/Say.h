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

#pragma once

#include <ros/ros.h>

#include "twitcher_actions/RobotAction.h"
#include "twitcher_actions/SayAction.h"

class Say : public RobotAction<twitcher_actions::SayAction,
                                        twitcher_actions::SayGoal,
                                        twitcher_actions::SayFeedback,
                                        twitcher_actions::SayResult>
{
public:
    Say();
    Say(ros::NodeHandle node);
    ~Say();
protected:
    virtual void executeAction(const twitcher_actions::SayGoal::ConstPtr& goal);
    
    ros::Publisher soundplay_publisher;
};

