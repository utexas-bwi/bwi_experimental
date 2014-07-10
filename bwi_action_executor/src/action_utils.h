
#ifndef bwi_action_executor_action_utils_h__guard
#define bwi_action_executor_action_utils_h__guard

#include "actions/Action.h"

struct ActionToAsp {
	
	std::string operator()(bwi_actexec::Action *a) {
		return a->toASP(counter++);
	}
	unsigned int counter;
};

struct DeleteAction {
	void operator()(bwi_actexec::Action *a) {
		delete a;
	}
};

struct CloneAction {
	bwi_actexec::Action * operator()(bwi_actexec::Action * a) {
		return a->clone();
	}
};

#endif
