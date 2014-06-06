#include "OpenDoor.h"

#include "ActionFactory.h"

#include "../kr_interface.h"

#include "CallGUI.h"
#include "LogicalNavigation.h"
#include <boost/concept_check.hpp>

#include <iostream>

using namespace std;

namespace bwi_actexec {
	
OpenDoor::OpenDoor(const string& name) : name(name) {}

	
void OpenDoor::run() {

	if(!asked) {
		CallGUI askToOpen("askToOpen", CallGUI::DISPLAY,  "Can you open door " + doorName + ", please?");
		askToOpen.run();
		asked = true;
	}
	
	if(!doorOpen) {
		LogicalNavigation senseDoor("sensedoor");
		vector<string> params;
		params.push_back(doorName);
		
		senseDoor.init(params);
		
		senseDoor.run();
		
		//check if door is open
		bwi_kr::AnswerSetMsg answerSet = kr_query("open(" + doorName + ",0).",0);

		doorOpen = answerSet.satisfied;
		
		cerr << "door open: " << doorOpen << endl;
	}
	
	if(doorOpen) {
		CallGUI askToOpen("thank", CallGUI::DISPLAY,  "Thanks!");
		askToOpen.run();
	}
	
}

void OpenDoor::init(const vector< string >& params) {
	doorName = params[0];
}


vector< string > OpenDoor::getParameters() const {
	vector<string> params;
	params.push_back(doorName);
	return params;
}

ActionFactory openDoorAction(new OpenDoor("opendoor"));

	
	
	
}