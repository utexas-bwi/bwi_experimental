

#include "RemoteReasoner.h"

#include "actasp/reasoners/Clingo.h"
#include "actasp/executors/ReplanningActionExecutor.h"
#include "actasp/ActionExecutor.h"
#include "actasp/AspFluent.h"
#include "actasp/planners/AnyPlan.h"

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

  ActionFactory::setSimulation(true); //parametrize this

  AspKR *reasoner = new RemoteReasoner();
  Planner *planner = new AnyPlan(reasoner);
  ActionExecutor *executor = new ReplanningActionExecutor(reasoner,planner,ActionFactory::actions());
  
  vector<AspRule> goalRules;
  
  goalRules.push_back(AspRule() << AspFluent("not visited(l3_418,n)"));
  goalRules.push_back(AspRule() << AspFluent("not visited(l3_414a,n)"));
  
  executor->setGoal(goalRules);
  
  ros::Rate loop(10);
  
  while (ros::ok() && !executor->goalReached() && !executor->failed()) {
    
    executor->executeActionStep();

    ros::spinOnce();

    loop.sleep();
  }
  
  delete executor;
  delete reasoner;

  return 0;
}