/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Texas at Austin 
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of UT Austin nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Note that this code is based on the SimpleActionServer class, and 
* the original copyright notice included below.
*
*/

/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

namespace actionlib {

  template <class ActionSpec>
  InterruptableActionServer<ActionSpec>::InterruptableActionServer(ros::NodeHandle n, std::string name) : n_(n) {

    std::string interruptable_server_name = name + "_interruptable"

    // create the action server.
    as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n_, interruptable_server_name,
          boost::bind(&InterruptableActionServer::goalCallback, this, _1),
          boost::bind(&InterruptableActionServer::cancelCallback, this, _1),
          false));

    // create the pause and resume services.
    pause_server_ = n_.advertiseService(interruptable_server_name + "/pause", &InterruptableActionServer::pause, this);
    resume_server_ = n_.advertiseService(interruptable_server_name + "/resume", &InterruptableActionServer::resume, this);

    // Create the lower level simple action client to the uninterruptable action server.
    ac_ = boost::shared_ptr<actionlib::SimpleActionClient<ActionSpec> >(new actionlib::SimpleActionClient<ActionSpec>(name, true));
  }

  template <class ActionSpec>
  bool InterruptableActionServer<ActionSpec>::pause() {
    boost::recursive_mutex::scoped_lock lock(lock_);
    bool ret_val = true;
    if (original_goal_available_) {
      ROS_ERROR_STREAM("InterruptableActionServer : Already paused one goal, cannot pause another.");
      ret_val = false;
    } else if (!pursue_current_goal_) {
      ROS_ERROR_STREAM("InterruptableActionServer : Not currently actively pursuing a goal, cannot pause.");
      ret_val = false;
    } else {
      pursue_current_goal_ = false;
      original_goal_ = current_goal_;
      original_goal_available_ = true;
    }
    return ret_val;
  }

  template <class ActionSpec>
  bool InterruptableActionServer<ActionSpec>::resume() {
    boost::recursive_mutex::scoped_lock lock(lock_);
    bool ret_val = true;
    if (!original_goal_available_) {
      ROS_ERROR_STREAM("InterruptableActionServer : No paused goal available, cannot resume goal.");
      ret_val = false;
    } else {
      // Restart the current goal again.
      switch_to_original_goal_ = true;
    }
    paused_ = false;
  }

  template <class ActionSpec>
  void InterruptableActionServer<ActionSpec>::goalCallback(GoalHandle goal) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    //check that the timestamp is past or equal to that of the current goal and the next goal
    if((!current_goal_.getGoal() || goal.getGoalID().stamp >= current_goal_.getGoalID().stamp)
        && (!next_goal_.getGoal() || goal.getGoalID().stamp >= next_goal_.getGoalID().stamp)) {

      //if next_goal has not been accepted already... its going to get bumped, but we need to let the client know we're preempting
      if(next_goal_.getGoal() && (!current_goal_.getGoal() || next_goal_ != current_goal_)) {
        next_goal_.setCanceled(Result(), "This goal was canceled because another goal was recieved by the simple action server");
      }

      next_goal_ = goal;
      next_goal_available_ = true; 
      
      // Trigger runLoop to call execute()
      execute_condition_.notify_all();
    } else {
      goal.setCanceled(Result(), "This goal was canceled because another goal was recieved by the simple action server");
    }

  }
  
  template <class ActionSpec>
  void InterruptableActionServer<ActionSpec>::cancelCallback(GoalHandle goal) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (goal == current_goal_) {
      pursue_current_goal_ = false;
    } else if (goal == next_goal_) {
      next_goal_.setCanceled();
      next_goal_available_ = false;
    } else if (goal == original_goal_ && original_goal_available_) {
      // Don't pursue the original goal again.
      original_goal_available_ = false;
    }
  }


  {
    while (n_.ok()) {
      boost::recursive_mutex::scoped_lock lock(lock_);
      if (switch_to_original_goal_) {
        if (pursue_current_goal_) {
          current_goal_.setCanceled();
        }
        if (next_goal_available_) {
          next_goal_available_ = false;
          next_goal_.setCanceled();
        }
        current_goal_ = original_goal_;
        original_goal_available_ = false;
        switch_to_original_goal_ = false;
        pursue_current_goal_ = true;
      } else if (next_goal_available_) {
        if (pursue_current_goal_) {
          current_goal_.setCanceled();
        }
        current_goal_ = next_goal_;
        next_goal_available_ = false;
      } else if (pursue_current_goal_) {
        // Make call to action client to check status of current goal. Publish feedback 

      }
    }
  }

  template <class ActionSpec>
  void InterruptableActionServer<ActionSpec>::start() {
    as_->start();
  }

};

