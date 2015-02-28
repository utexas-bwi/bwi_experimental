#include "CallElevator.h"

#include "ActionFactory.h"

#include "ros/console.h"


namespace bwi_krexec {

void CallElevator::run() {
  
  ROS_INFO("changefloor invoked");
  
}

bool CallElevator::hasFinished() const {
  //TODO implement this!
  return true;
}

bool CallElevator::hasFailed() const {
  //TODO implement this if this action can fail
  return false;
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
ActionFactory changeFloor(new CallElevator());

}
