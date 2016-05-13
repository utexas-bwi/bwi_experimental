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

#include "twitcher_actions/facedoor.h"

#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

FaceDoor::FaceDoor()
    : RobotAction<twitcher_actions::FaceDoorAction,
                  twitcher_actions::FaceDoorGoal,
                  twitcher_actions::FaceDoorFeedback,
                  twitcher_actions::FaceDoorResult>()
{

}

FaceDoor::FaceDoor(ros::NodeHandle node)
    : RobotAction<twitcher_actions::FaceDoorAction,
                  twitcher_actions::FaceDoorGoal,
                  twitcher_actions::FaceDoorFeedback,
                  twitcher_actions::FaceDoorResult>("FaceDoor", node)
{
    ROS_INFO_STREAM("/FaceDoor action server set up");
}

FaceDoor::~FaceDoor()
{

}

void FaceDoor::executeAction(const twitcher_actions::FaceDoorGoal::ConstPtr& goal)
{
    twitcher_actions::FaceDoorResult result;

    // Wait for action executor server to start up
    Client client("action_executor/execute_plan", true);
    client.waitForServer();
    
    bwi_kr_execution::ExecutePlanGoal actionGoal;
  
  
    // Matteo-created code that has saved us
    // Create ASP Rule to determine completion of goal
    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not facing";
    
    fluent.variables.push_back(goal->door_name);
    
    rule.body.push_back(fluent);
    actionGoal.aspGoal.push_back(rule);
    
    
    
    ROS_INFO_STREAM("Sending goal: Going to location " << goal->door_name);
    client.sendGoal(actionGoal);
    
    ros::Rate wait_rate(10);
    while(ros::ok() && !client.getState().isDone())
        wait_rate.sleep();
    
        
    if (!client.getState().isDone()) {
        ROS_INFO("Canceling goal since it is not done");
        client.cancelGoal();
        //and wait for canceling confirmation
        for(int i = 0; !client.getState().isDone() && i<50; ++i)
        wait_rate.sleep();
    }
    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        result.success = false;
        server->setAborted(result);
        ROS_INFO("Aborted");
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        result.success = false;
        server->setPreempted(result);
        ROS_INFO("Preempted");
    }
    
    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Succeeded!");
        result.success = true;
        server->setSucceeded(result);
    }
    else {
        result.success = false;
        server->setAborted(result);
        ROS_INFO("Terminated");
    }
}

