#ifndef bwi_actexec_ActionFactory_h__guard
#define bwi_actexec_ActionFactory_h__guard

#include "Action.h"

#include <string>
#include <map>
#include <stdexcept>

namespace bwi_actexec {

class ActionFactory {


public:
	
	ActionFactory(Action *act);
	
	static Action *byName(const std::string& name) throw (std::runtime_error);
	static std::map<std::string, Action*> &actions();
};

}

#endif