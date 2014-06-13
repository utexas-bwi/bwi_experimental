#include "ActionFactory.h"

using namespace std;

namespace bwi_actexec {
	
ActionFactory::ActionFactory(Action* act) {

	//TODO warn if action already there
	actions().insert( make_pair(act->getName(),act));

}


Action* ActionFactory::byName(const std::string& name) throw (std::runtime_error) {
	
	map<string, Action*>::const_iterator actIt = actions().find(name);
	
	if(actIt == actions().end())
		throw runtime_error("No action with name " + name);
	
	return actIt->second->clone();
}

std::map<std::string, Action*> &ActionFactory::actions() {
	static std::map<std::string, Action*> acts;
	return acts;
}

	
}