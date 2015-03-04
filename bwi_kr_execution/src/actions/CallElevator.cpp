#include "CallElevator.h"

#include "ActionFactory.h"

#include "ros/console.h"


namespace bwi_krexec {

void CallElevator::run() {
  
  if(!asked) {
    std::string direction_text = (going_up) ? "up" : "down";
    doors.clear()
    // TODO - get door names from current_state_query here. Not sure how.
    askToCallElevator.reset(new CallGUI("askToCallElevator", CallGUI::CHOICE_QUESTION,  "Could you call elevator " + elevator + " to go " + direction_text + ", and then let me know which door opened?", 60.0f, doors);
    askToCallElevator->run();
    asked = true;
    startTime = ros::Time::now();
  }
  
  if(!done) {
    if (askToCallElevator->hasFinished()) {
      // Check response to see it's positive.
      int response_idx = askToCallElevator->getResponseIndex();
      if (response_idx >= 0 && response_idx < doors.size()) {
        // TODO - Update current state with this door being open.
        ROS_DEBUG_STREAM( "door open: " << open );
        CallGUI thanks("thanks", CallGUI::DISPLAY,  "Thanks!");
        thanks.run();
      } else {
        // A door didn't open in the timeout specified.
        failed = true;
      }
      done = true;
    }
  }
  
}

bool CallElevator::hasFinished() const {
  return done;
}

bool CallElevator::hasFailed() const {
  return failed;
}

actasp::Action *CallElevator::cloneAndInit(const actasp::AspFluent & fluent) const {
  CallElevator *other = new CallElevator();
  other->elevator = fluent.getParameters().at(0);
  other->going_up = fluent.getParameters().at(1) == "up"; //throws an exception if the parameter doesn't exist
  return other;
}

std::vector<std::string> CallElevator::getParameters() const {
  std::vector<std::string> params;
  params.reserve(2);
  params.push_back(elevator);
  params.push_back(going_up? "up" : "down");
  return params;
}

//if you want the action to be available only in simulation, or only
//on the robot, use the constructor that also takes a boolean.
ActionFactory callElevator(new CallElevator());

}
