#include "AskInRoom.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "bwi_kr_execution/AspFluent.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include <bwi_kr_execution/UpdateFluents.h>

#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

AskInRoom::AskInRoom() : 
            person(),
            room(),
            pub_set(false),
            done(false){
            }

  
void AskInRoom::run() {
  //current state query
  ros::NodeHandle n;
  if (!pub_set) { 
    ask_pub = n.advertise<sound_play::SoundRequest>("robotsound", 1000);
    pub_set = true;
  }

  ros::ServiceClient currentClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
  
  bwi_kr_execution::AspFluent atFluent;
  atFluent.name = "at";
  atFluent.timeStep = 0;
  atFluent.variables.push_back(room);
  
  bwi_kr_execution::AspRule rule;
  rule.head.push_back(atFluent);
  
  bwi_kr_execution::CurrentStateQuery csq;
  csq.request.query.push_back(rule);
  
  currentClient.call(csq);
  
  bool at = csq.response.answer.satisfied;

  if (at) {
    //sound generation
    sound_play::SoundRequest sound_req;
    sound_req.sound = sound_play::SoundRequest::SAY;
    sound_req.command = sound_play::SoundRequest::PLAY_ONCE;
    std::stringstream ss;
    ss << "Is " << person << "in the room?";
    sound_req.arg = ss.str();
    ask_pub.publish(sound_req);
    ROS_INFO("%d", ask_pub.getNumSubscribers());
    ROS_INFO_STREAM(ask_pub.getTopic());

    /*sound_play::SoundClient client(n,"robotsound");
    client.say("Is " + person + " inside the room");*/
  }

  vector<string> options;
  options.push_back("yes");
  options.push_back("no");

  CallGUI askInRoom("askInRoom", CallGUI::CHOICE_QUESTION,  "Is " + person + " inside the room " + room + "?", 20.0, options);
  askInRoom.run();

  int response = askInRoom.getResponseIndex();

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
