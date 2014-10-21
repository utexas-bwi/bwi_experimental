#include "AskInRoom.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "bwi_kr_execution/AspFluent.h"

#include <bwi_kr_execution/UpdateFluents.h>

#include <ros/ros.h>

#include <iostream>

using namespace std;

namespace bwi_krexec {

AskInRoom::AskInRoom() : 
            person(),
            room(),
            done(false){}

  
void AskInRoom::run() {
  vector<string> options;
  options.push_back("yes");
  options.push_back("no");

  CallGUI askInRoom("askInRoom", CallGUI::CHOICE_QUESTION,  "Is " + person + " inside the room " + room + "?", 20.0, options);
  askInRoom.run();

  int response = askInRoom.getResponseIndex();

  ros::NodeHandle n;
  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
  krClient.waitForExistence();
  bwi_kr_execution::UpdateFluents uf;

  bwi_kr_execution::AspFluent fluent;
  fluent.timeStep = 0;
  fluent.variables.push_back(person);
  fluent.variables.push_back(room);

  fluent.name = ((response == 0) ? "inroom" : "-inroom");

  uf.request.fluents.push_back(fluent);
  krClient.call(uf);

  if (response >= 0) {
    CallGUI thank("thank", CallGUI::DISPLAY,  "Thanks!");
    thank.run();
  }

  done = true;

}  
  
actasp::Action* AskInRoom::cloneAndInit(const actasp::AspFluent& fluent) const {
  AskInRoom *newAction = new AskInRoom();
  newAction->person = fluent.getParameters().at(0);
  newAction->room = fluent.getParameters().at(1);
  
  return newAction;
}

std::vector<std::string> AskInRoom::getParameters() const {
  vector<string> param;
  param.push_back(person);
  param.push_back(room);
  return param;
}


ActionFactory AskInRoomFactory(new AskInRoom());
  
}
