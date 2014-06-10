
#include "Order.h"

#include "ActionFactory.h"

#include <bwi_planning_common/PlannerInterface.h>
#include <bwi_kr/ChangeFluent.h>
#include "CallGUI.h"

#include <ros/ros.h>

#include <sstream>
using namespace ros;
using namespace std;

namespace bwi_actexec{

Order::Order(const std::string& name) :
			name(name),
			parameters(),
			done(false){}

void Order::init(const vector< string >& params) {
	this->parameters = params;
}

void Order::run(){

	vector< string > options;
	CallGUI loadGUI("LoadGUI", CallGUI::DISPLAY,
			"Could I get an order of "+ parameters[0] +"?",
			  0.0 );
	loadGUI.run();
	sleep(5.0);


	CallGUI clearGUI("askToOpen", CallGUI::DISPLAY,"");
	clearGUI.run();

    // Pass along "fake" observation to KR node
    NodeHandle n;
    std::vector<bwi_planning_common::PlannerAtom> observations;
    bwi_planning_common::PlannerAtom observation1;
    observation1.name = "waiting";
    observation1.value.push_back(parameters[0]);
    observations.push_back(observation1);
    ros::ServiceClient krClient = n.serviceClient<bwi_kr::ChangeFluent> ( "/bwi_kr/change_fluent" );
    krClient.waitForExistence();
    for(int i=0, size=observations.size() ; i < size; ++i) {
        bwi_kr::ChangeFluent cf;
        cf.request.fluent.name = observations[i].name;
        cf.request.fluent.parameters = observations[i].value;
        krClient.call(cf);
    }

	done = true;
}

static ActionFactory orderFactory(new Order("order"));

}
