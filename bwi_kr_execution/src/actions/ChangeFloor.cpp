#include "ChangeFloor.h"

#include "ActionFactory.h"

#include "ros/console.h"


namespace bwi_krexec {

void ChangeFloor::run() {
  
  ROS_INFO("changefloor invoked");
  
}

bool ChangeFloor::hasFinished() const {
  //TODO implement this!
  return true;
}

bool ChangeFloor::hasFailed() const {
  //TODO implement this if this action can fail
  return false;
}

actasp::Action *ChangeFloor::cloneAndInit(const actasp::AspFluent & fluent) const {
  ChangeFloor *other = new ChangeFloor();
  other->dest_room = fluent.getParameters().at(0);
  return other;
}

std::vector<std::string> ChangeFloor::getParameters() const {
  return std::vector<std::string>(1,dest_room);
}

//if you want the action to be available only in simulation, or only
//on the robot, use the constructor that also takes a boolean.
ActionFactory changeFloor(new ChangeFloor());

}
