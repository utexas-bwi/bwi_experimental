#include "LogicalNavigation.h"

#include "ActionFactory.h"

#include <bwi_planning_common/PlannerInterface.h>
#include <bwi_kr/ChangeFluent.h>

#include <ros/ros.h>

#include <sstream>

using namespace ros;
using namespace std;

namespace bwi_actexec {
	
LogicalNavigation::LogicalNavigation(const std::string& name) :
			name(name),
			parameters(),
			done(false){}

	
void LogicalNavigation::init(const vector< string >& params) {
	this->parameters = params;
}

void LogicalNavigation::run() {

	NodeHandle n;
	ros::ServiceClient navClient = n.serviceClient<bwi_planning_common::PlannerInterface> ( "execute_logical_goal" );
	navClient.waitForExistence();
	
	bwi_planning_common::PlannerInterface pi;
	
	pi.request.command.name = name;
	pi.request.command.value = parameters;
	
	navClient.call(pi);
	
	ros::ServiceClient krClient = n.serviceClient<bwi_kr::ChangeFluent> ( "/bwi_kr/change_fluent" );
	//krClient.waitForExistence();
	
		//bwi_kr::ChangeFluent cf;
	//for(int i=0 ; i < pi.response.observations.size(); ++i) {
        //bwi_kr::Predicate fluent_single ;
		//fluent_single.name = pi.response.observations[i].name;
		//fluent_single.parameters = pi.response.observations[i].value;
        //cf.request.fluent.push_back(fluent_single);
	//}
		//krClient.call(cf);
       
		//bwi_kr::ChangeFluent cf;

	//for(int i=0 ; i < pi.response.observations.size(); ++i) {
		//cf.request.fluent.name = pi.response.observations[i].name;
		//cf.request.fluent.parameters = pi.response.observations[i].value;
//        //cf.request.fluent.push_back(fluent_single);
	//}
		//krClient.call(cf);
		krClient.waitForExistence();

	for(int i=0, size=pi.response.observations.size() ; i < size; ++i) {

		bwi_kr::ChangeFluent cf;

		cf.request.fluent.name = pi.response.observations[i].name;
		cf.request.fluent.parameters = pi.response.observations[i].value;

		krClient.call(cf);
	}
	
	done = true;
	
}

static ActionFactory goToFactory(new LogicalNavigation("goto"));
static ActionFactory gothroughFactory(new LogicalNavigation("gothrough"));
static ActionFactory approachFactory(new LogicalNavigation("approach"));
	
} //namespace
