
#include "actasp/reasoners/Clingo.h"
#include "actasp/AspFluent.h"

#include "actions/ActionFactory.h"
#include "actions/LogicalNavigation.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>

#include <string>


using namespace std;
using namespace bwi_krexec;
using namespace actasp;

int main(int argc, char**argv) {
  ros::init(argc, argv, "bwi_action_execution");
  ros::NodeHandle n;
  
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

//  create initial state
  LogicalNavigation setInitialState("noop");
  setInitialState.run();




  return 0;
}