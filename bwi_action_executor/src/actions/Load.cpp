
#include "Load.h"

#include "ActionFactory.h"

#include <bwi_planning_common/PlannerInterface.h>
#include <bwi_kr/ChangeFluent.h>
#include "CallGUI.h"

#include <ros/ros.h>

#include <sstream>
using namespace ros;
using namespace std;

namespace bwi_actexec{

Load::Load(const std::string& name) :
			name(name),
			parameters(),
			done(false){}

void Load::init(const vector< string >& params) {
	this->parameters = params;
}

void Load::run(){
	vector< string > options;
	options.push_back("Done!");
	CallGUI loadGUI("LoadGUI", CallGUI::CHOICE_QUESTION,
			  "Please let me know once " +
			  this->parameters[0] + " has been loaded!",
			  0.0,  options );
	loadGUI.run();


	CallGUI clearGUI("askToOpen", CallGUI::DISPLAY,"");
	clearGUI.run();


	NodeHandle n;
    std::vector<bwi_planning_common::PlannerAtom> observations;
    bwi_planning_common::PlannerAtom observation1;
    observation1.name = "loaded";
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

static ActionFactory loadFactory(new Load("load"));

}
