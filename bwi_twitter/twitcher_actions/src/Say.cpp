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

#include "twitcher_actions/Say.h"
#include <sound_play/SoundRequest.h>

Say::Say()
    : RobotAction<twitcher_actions::SayAction,
                    twitcher_actions::SayGoal,
                    twitcher_actions::SayFeedback,
                    twitcher_actions::SayResult>()
{
}

Say::Say(ros::NodeHandle node)
    : RobotAction<twitcher_actions::SayAction,
                  twitcher_actions::SayGoal,
                  twitcher_actions::SayFeedback,
                  twitcher_actions::SayResult>("SayTwitter", node)
{
    soundplay_publisher = node.advertise<sound_play::SoundRequest>("robotsound", 100);
    ROS_INFO("/SayTwitter action server up");
}

void Say::executeAction(const twitcher_actions::SayGoal::ConstPtr& goal)
{
    twitcher_actions::SayResult result;
    std::string spoken_text = goal->message;
     
    sound_play::SoundRequest msg;
    msg.sound = sound_play::SoundRequest::SAY;
    msg.command = sound_play::SoundRequest::PLAY_ONCE;
    msg.arg = spoken_text;
    
    soundplay_publisher.publish(msg);

    result.success = true;
    server->setSucceeded(result);
}


Say::~Say()
{
    
}
