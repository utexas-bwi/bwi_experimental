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
*********************************************************************/

#ifndef BWI_INTERRUPTABLE_ACTION_SERVER_H_
#define BWI_INTERRUPTABLE_ACTION_SERVER_H_

#include <boost/thread/condition.hpp>
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/action_definition.h>

namespace actionlib {
  template <class ActionSpec>
  class InterruptableActionServer {
    public:
      typedef typename ActionServer<ActionSpec>::GoalHandle GoalHandle;

      InterruptableActionServer(ros::NodeHandle n, std::string name);

      ~InterruptableActionServer();

      void start();

    private:

      bool pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      bool resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      void goalCallback(GoalHandle goal);
      void cancelCallback(GoalHandle preempt);

      ros::NodeHandle n_;

      ros::ServiceServer pause_server_;
      ros::ServiceServer resume_server_;

      boost::shared_ptr<ActionServer<ActionSpec> > as_;
      boost::shared_ptr<SimpleActionClient<ActionSpec> > ac_;

  };
};

//include the implementation here
#include <actionlib/server/interruptable_action_server_imp.h>
#endif
