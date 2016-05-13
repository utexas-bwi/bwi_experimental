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

#ifndef TWITCHER_ROBOTACTION_H
#define TWITCHER_ROBOTACTION_H

#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

template <typename Action, typename Goal, typename Feedback, typename Result>
class RobotAction {
public:
    RobotAction() {
    }
    
    RobotAction(std::string server_name, ros::NodeHandle node)
      : node(node)
    {
        server = new actionlib::SimpleActionServer<Action>(node,
                                                           server_name, 
                                                           boost::bind(&RobotAction<Action, Goal, Feedback, Result>::executeAction, this, _1),
                                                           false);
        server->start();
    }
    
    virtual ~RobotAction() { delete server; }
protected:
    virtual void executeAction(const typename Goal::ConstPtr &goal) = 0;
    
    ros::NodeHandle node;
    actionlib::SimpleActionServer<Action>* server;
};


#endif // #ifndef TWITCHER_ROBOTACTION_H