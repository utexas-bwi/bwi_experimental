#include "AutomatedAskPerson.h"
#include "ActionFactory.h"
#include "AutomatedPersonLocation.h"

#include "bwi_kr_execution/AspFluent.h"
#include <bwi_kr_execution/UpdateFluents.h>

#include <ros/ros.h>
#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

AutomatedAskPerson::AutomatedAskPerson() : 
            done(false),
            started(false) {
            }

std::map<std::string, std::string> AutomatedAskPerson::person_location_map;
bool AutomatedAskPerson::person_location_available(false);
  
void AutomatedAskPerson::run() {

  if (!person_location_available) {
    readAutomatedPersonLocation(person_location_map);
    person_location_available = true;
  }

  if (!started) {
    startTime = ros::Time::now();
    started = true;
  } else {
    if ((ros::Time::now() - startTime) > ros::Duration(15.0)) {

      // Update fluents to indicate the effect of this action.
      ros::NodeHandle n;
      ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
      krClient.waitForExistence();

      bwi_kr_execution::AspFluent fluent;
      fluent.timeStep = 0;
      fluent.name = "inroom";
      fluent.variables.push_back(person_to_know);
      fluent.variables.push_back(person_location_map[person_to_know]);

      bwi_kr_execution::UpdateFluents uf;
      uf.request.fluents.push_back(fluent);

      krClient.call(uf);

      done = true;
    }
  }

}  
  
actasp::Action* AutomatedAskPerson::cloneAndInit(const actasp::AspFluent& fluent) const {
  AutomatedAskPerson *newAction = new AutomatedAskPerson();
  newAction->person_to_ask = fluent.getParameters().at(0);
  newAction->person_to_know = fluent.getParameters().at(1);
  
  return newAction;
}

std::vector<std::string> AutomatedAskPerson::getParameters() const {
  vector<string> param;
  param.push_back(person_to_ask);
  param.push_back(person_to_know);
  return param;
}


ActionFactory automatedAskPersonFactory(new AutomatedAskPerson(), true);
  
}
