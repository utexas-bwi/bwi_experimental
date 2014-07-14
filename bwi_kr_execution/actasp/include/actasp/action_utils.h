
#ifndef actasp_action_util_h__guard
#define actasp_action_util_h__guard

#include <actasp/Action.h>

#include <utility>

namespace actasp {
	
struct ActionDeleter {
	void operator()(Action *act) const {
		delete act;
	}
};

struct ActionMapDeepCopy {
	
	std::pair<std::string, Action *> operator()(const std::pair<std::string, Action *> &other) {
		
		return std::make_pair(other.first, other.second->clone());
	}
};

struct ActionMapDelete {
	
	void operator()(const std::pair<std::string, Action *> &other) {
		delete other.second;
	}
};

}


#endif