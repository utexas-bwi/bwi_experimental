#ifndef TaskInstantiator_h__guard
#define TaskInstantiator_h__guard

#include "bwi_kr_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>

#include <pnp/pnp_instantiators.h>
#include <pnp/basic_plan/xml_plan_instantiator.h>
#include <pnp/pnpfwd.h>

#include "TaskCondition.h"


struct TaskInstantiator : public PetriNetPlans::ExecutableInstantiator {
	
    explicit TaskInstantiator(const std::string& planDir, Client& client);
	
    PetriNetPlans::PnpExecutable* createExecutable(const std::string& name) throw(std::runtime_error);
	
private:
    std::string planDir;
    TaskCondition condition;
    Client& client;
    PetriNetPlans::XMLPnpPlanInstantiator planLoader;

    PetriNetPlans::PnpExecutable* createAction(const std::string& name);
	
};


#endif
