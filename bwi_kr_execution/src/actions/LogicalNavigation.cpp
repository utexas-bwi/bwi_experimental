#include "LogicalNavigation.h"

#include "ActionFactory.h"

#include "actasp/AspFluent.h"

#include <bwi_planning_common/PlannerInterface.h>
#include <bwi_kr_execution/UpdateFluents.h>
#include <bwi_kr_execution/AspFluent.h>

#include <ros/ros.h>

#include <sstream>

using namespace ros;
using namespace std;
using namespace actasp;

namespace bwi_krexec {
	
LogicalNavigation::LogicalNavigation(const std::string& name, const std::vector<std::string>& parameters) :
			name(name),
			parameters(parameters),
			done(false){}


struct PlannerAtom2AspFluent {
  bwi_kr_execution::AspFluent operator()(const bwi_planning_common::PlannerAtom& atom) {
    
    bwi_kr_execution::AspFluent fluent;
    fluent.name = atom.name;
    if(!atom.value.empty()) {
      fluent.variables.insert(fluent.variables.end(),atom.value.begin(),atom.value.end());
      
      fluent.timeStep = 0; //the observation does not provide a timeStep
    }
    
    return fluent;
  }
};
    

void LogicalNavigation::run() {

	NodeHandle n;
	ros::ServiceClient navClient = n.serviceClient<bwi_planning_common::PlannerInterface> ( "execute_logical_goal" );
	navClient.waitForExistence();
	
	bwi_planning_common::PlannerInterface pi;
	
	pi.request.command.name = name;
	pi.request.command.value = parameters;
	
	navClient.call(pi);
        

	ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
	krClient.waitForExistence();
  
  bwi_kr_execution::UpdateFluents uf;
  
  transform(pi.response.observations.begin(),pi.response.observations.end(),back_inserter(uf.request.fluents),PlannerAtom2AspFluent());

	krClient.call(uf);
	
	done = true;
	
}

Action *LogicalNavigation::cloneAndInit(const actasp::AspFluent & fluent) const {
  return new LogicalNavigation(fluent.getName(),fluent.getParameters());
}



static ActionFactory gothroughFactory(new LogicalNavigation("gothrough"));
static ActionFactory approachFactory(new LogicalNavigation("approach"));
	
} //namespace
