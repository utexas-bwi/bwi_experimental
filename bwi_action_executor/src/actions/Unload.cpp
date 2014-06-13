
#include "Unload.h"

#include "ActionFactory.h"

#include <bwi_planning_common/PlannerInterface.h>
#include <bwi_kr/ChangeFluent.h>
#include "CallGUI.h"

#include <ros/ros.h>

#include <sstream>
using namespace ros;
using namespace std;

namespace bwi_actexec{

Unload::Unload(const std::string& name) :
			name(name),
			parameters(),
			done(false){}

void Unload::init(const vector< string >& params) {
	this->parameters = params;
}

void Unload::run(){
	vector< string > options;
	options.push_back("Done!");
	CallGUI loadGUI("LoadGUI", CallGUI::CHOICE_QUESTION,
			"Hello "+ parameters[1] +", here is your "
			+ this->parameters[0]
			+ "! Please let me know once you have removed it.",
			  0.0,  options );
	loadGUI.run();


	CallGUI clearGUI("askToOpen", CallGUI::DISPLAY,"");
	clearGUI.run();

    // Pass along "fake" observation to KR node
    NodeHandle n;
    std::vector<bwi_planning_common::PlannerAtom> observations;
    bwi_planning_common::PlannerAtom observation1;
    observation1.name = "served";
    observation1.value.push_back(parameters[1]);
    observation1.value.push_back(parameters[0]);
    observations.push_back(observation1);
    ros::ServiceClient krClient = n.serviceClient<bwi_kr::ChangeFluent> ( "/bwi_kr/change_fluent" );
    krClient.waitForExistence();
        bwi_kr::ChangeFluent cf;
    for(int i=0, size=observations.size() ; i < size; ++i) {
		cf.request.fluent.name =observations[i].name;
		cf.request.fluent.parameters = observations[i].value;
    }
        krClient.call(cf);

	done = true;
}

static ActionFactory unloadFactory(new Unload("unloadto"));

}
