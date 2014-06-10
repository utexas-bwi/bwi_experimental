
#include "Greet.h"

#include "ActionFactory.h"

#include <bwi_planning_common/PlannerInterface.h>
#include <bwi_kr/ChangeFluent.h>
#include "CallGUI.h"

#include <ros/ros.h>

#include <sstream>
using namespace ros;
using namespace std;

namespace bwi_actexec{

Greet::Greet(const std::string& name) :
			name(name),
			parameters(),
			done(false){}

void Greet::init(const vector< string >& params) {
	this->parameters = params;
}

void Greet::run(){
    // Write to Screen
    vector< string > options;
    options.push_back("Continue");
    CallGUI loadGUI("LoadGUI", CallGUI::CHOICE_QUESTION,
            "If you are "+ parameters[0] +" please click continue.",
              0.0,  options );
    loadGUI.run();

    // Clear GUI
	CallGUI clearGUI("askToOpen", CallGUI::DISPLAY,"");
	clearGUI.run();

    // Pass along "fake" observation to KR node
    NodeHandle n;
    std::vector<bwi_planning_common::PlannerAtom> observations;
    bwi_planning_common::PlannerAtom observation1;
    observation1.name = "closeto";
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

static ActionFactory greetFactory(new Greet("greet"));

}
