#include "OpenSimulatedDoor.h"

#include "segbot_simulation_apps/DoorHandlerInterface.h"

#include "ActionFactory.h"
#include "LogicalNavigation.h"

#include "actasp/AspFluent.h"

#include <ros/ros.h>

using namespace std;
using namespace ros;

namespace bwi_krexec {


OpenSimulatedDoor::OpenSimulatedDoor() : door(), done(false) {}

void OpenSimulatedDoor::run() {
  NodeHandle n;
  ServiceClient doorClient = n.serviceClient<segbot_simulation_apps::DoorHandlerInterface> ( "update_doors" );
  doorClient.waitForExistence();
  
  segbot_simulation_apps::DoorHandlerInterface dhi;
  
  dhi.request.all_doors = false;
  dhi.request.door = door;
  dhi.request.open = true;
  
  doorClient.call(dhi);

  vector<string> params;
  params.push_back(door);
  LogicalNavigation senseDoor("sensedoor",params);

  senseDoor.run();
  
  done = true;
}


actasp::Action* OpenSimulatedDoor::cloneAndInit(const actasp::AspFluent& fluent) const {
  OpenSimulatedDoor *newAction = new OpenSimulatedDoor();
  newAction->door = fluent.getParameters().at(0);

  return newAction;
}

std::vector<std::string> OpenSimulatedDoor::getParameters() const {
  vector<string> param;
  param.push_back(door);
  return param;
}

bwi_krexec::ActionFactory simulatedOpenDoor(new OpenSimulatedDoor(), true);

}