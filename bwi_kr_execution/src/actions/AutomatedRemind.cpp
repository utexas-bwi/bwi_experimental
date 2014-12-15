#include "AutomatedRemind.h"

#include "ActionFactory.h"

#include "bwi_kr_execution/AspFluent.h"

#include <bwi_kr_execution/UpdateFluents.h>

#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

AutomatedRemind::AutomatedRemind() : 
            done(false),
            started(false) {}

void AutomatedRemind::run() {

  if (!started) {
    startTime = ros::Time::now();
    started = true;
  } else {
    if ((ros::Time::now() - startTime) > ros::Duration(15.0)) {

      ros::NodeHandle n;
      ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
      krClient.waitForExistence();

      bwi_kr_execution::AspFluent fluent;
      fluent.timeStep = 0;
      fluent.name = "inmeeting";
      fluent.variables.push_back(person_to_remind);
      fluent.variables.push_back(meeting);

      bwi_kr_execution::UpdateFluents uf;
      uf.request.fluents.push_back(fluent);
      krClient.call(uf);

      done = true;
    }
  }
}

actasp::Action* AutomatedRemind::cloneAndInit(const actasp::AspFluent& fluent) const {
  AutomatedRemind *newAction = new AutomatedRemind();
  newAction->person_to_remind = fluent.getParameters().at(0);
  newAction->meeting = fluent.getParameters().at(1);
  newAction->room = fluent.getParameters().at(2);
  return newAction;
}

std::vector<std::string> AutomatedRemind::getParameters() const {
  vector<string> param;
  param.push_back(person_to_remind);
  param.push_back(meeting);
  param.push_back(room);
  return param;
}


ActionFactory automatedRemindFactory(new AutomatedRemind(), true);
  
}
