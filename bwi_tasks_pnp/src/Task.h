#ifndef TASK_H
#define TASK_H

#include "bwi_kr_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include <pnp/pnp_action.h>

using namespace std;
typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

class Task : public PetriNetPlans::PnpAction {
public:
    Task(bwi_kr_execution::ExecutePlanGoal &goal, Client &client);	
    virtual void executeStep();
    virtual bool finished();

private:
    bwi_kr_execution::ExecutePlanGoal goal;
    Client& client;
    bool goalSent;
    double initial_time;
};


#endif // TASK_H
