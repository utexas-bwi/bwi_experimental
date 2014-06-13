#ifndef bwi_actexec_ActionFactory_h__guard
#define bwi_actexec_ActionFactory_h__guard

#include "Action.h"

#include <string>
#include <map>
#include <stdexcept>

namespace bwi_actexec {

struct ActionFactory {
	
	typedef std::map<std::string, Action*> ActionMap;
	
	ActionFactory(Action *act);
	
	static Action *byName(const std::string& name) throw (std::runtime_error);
	static ActionMap &actions();
	
	
	
	
};

}

#endif