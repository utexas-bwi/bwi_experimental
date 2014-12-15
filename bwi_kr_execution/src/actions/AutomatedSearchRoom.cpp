#include "AutomatedSearchRoom.h"
#include "ActionFactory.h"
#include "AutomatedPersonLocation.h"

#include "bwi_kr_execution/AspFluent.h"
#include <bwi_kr_execution/UpdateFluents.h>

#include <ros/ros.h>
#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

AutomatedSearchRoom::AutomatedSearchRoom() : 
            done(false),
            started(false) {}

std::map<std::string, std::string> AutomatedSearchRoom::person_location_map;
bool AutomatedSearchRoom::person_location_available(false);
  
void AutomatedSearchRoom::run() {

  if (!person_location_available) {
    readAutomatedPersonLocation(person_location_map);
    person_location_available = true;
  }

  if (!started) {
    startTime = ros::time::now();
    started = true;
  } else {
    if ((ros::Time::now() - startTime) > ros::Duration(15.0)) {

      ros::NodeHandle n;
      ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
      krClient.waitForExistence();

      bwi_kr_execution::UpdateFluents uf;
      bwi_kr_execution::AspFluent fluent;
      fluent.timeStep = 0;
      fluent.variables.push_back(person);
      fluent.variables.push_back(room);

      bool response = (person_location_map[person] == room);
      fluent.name = ((response == 0) ? "inroom" : "-inroom");

      uf.request.fluents.push_back(fluent);
      krClient.call(uf);

      done = true;
    }
  }
}  
  
actasp::Action* AutomatedSearchRoom::cloneAndInit(const actasp::AspFluent& fluent) const {
  AutomatedSearchRoom *newAction = new AutomatedSearchRoom();
  newAction->person = fluent.getParameters().at(0);
  newAction->room = fluent.getParameters().at(1);
  return newAction;
}

std::vector<std::string> AutomatedSearchRoom::getParameters() const {
  vector<string> param;
  param.push_back(person);
  param.push_back(room);
  return param;
}


ActionFactory AutomatedSearchRoomFactory(new AutomatedSearchRoom(), true);
  
}
