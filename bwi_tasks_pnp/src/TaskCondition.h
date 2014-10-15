#ifndef TaskCondition_h__guard
#define TaskCondition_h__guard

#include <pnp/learning_plan/RewardCollector.h>

#include "bwi_kr_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

class TaskCondition : public learnpnp::RewardCollector {
public:
    TaskCondition(Client& client);
    bool evaluateAtomicExternalCondition(const std::string& atom);
    double reward();
    static void setReward(double reward);
private:
    Client& client;
    static double rew;
};


#endif
