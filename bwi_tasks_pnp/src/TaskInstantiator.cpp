#include "TaskInstantiator.h"

#include <pnp/pnp_action.h>
#include <pnp/pnp_plan.h>
#include <pnp/learning_plan/learnPlan.h>
#include <pnp/learning_plan/algo/BasicController.h>
#include <pnp/learning_plan/exp/EGreedy.h>
#include <pnp/learning_plan/algo/TDLambda.h>

#include <iostream>
#include <fstream>

#include "bwi_kr_execution/ExecutePlanAction.h"
#include <actasp/AspFluent.h>

#include "Task.h"
#include <ros/package.h>

#include <dirent.h>

using namespace std;
using namespace PetriNetPlans;

TaskInstantiator::TaskInstantiator(const std::string& planDir, Client& client) : planDir(planDir), condition(client), client(client), planLoader() {
}

PetriNetPlans::PnpExecutable* TaskInstantiator::createExecutable(const std::string& name) throw(std::runtime_error) {
    PnpExecutable *executable = createAction(name);
    if(executable != NULL)
        return executable;
	
    //otherwise, check if it is a plan
    PnpPlan *plan = new PnpPlan(this, &condition, name);


    string pnml = planDir + name + ".pnml";
    learnpnp::LearnPlan *lplan = NULL;

    try {
        planLoader.loadFromPNML (pnml, plan);
        ROS_INFO("plan loaded");
        learnpnp::TDLambda::ParamType params;
        params.alpha = 0.2;
        params.gamma = 0.99;
        params.initialValue = 0;
        params.lambda = 0.9;
	
        lplan = new learnpnp::LearnPlan(*plan, new learnpnp::BasicController(new learnpnp::TDLambda(ros::package::getPath("bwi_tasks_pnp")+"/plans/" + "value_function_" + name + ".txt", params), new learnpnp::EGreedy(0.1)));

    } catch ( const std::runtime_error& ) {		
        //no file for that plan
        string errorString ( "No action nor plan with name: " );
        errorString+=name;
        throw std::runtime_error ( errorString );
    }
    
    return lplan;

}

bwi_kr_execution::AspRule parseAspRule(const std::string &s) throw(std::runtime_error) {
    
    bwi_kr_execution::AspRule rule;

    int fluent_begin = s.find_first_not_of(":- ");
    int fluent_end = s.find(")", fluent_begin);

    string fl;    

    while (fluent_begin != string::npos) {
        if (fluent_end != string::npos) {
            fl = s.substr(fluent_begin, fluent_end-fluent_begin+1);
        }
        else throw std::runtime_error("The line '" + s + "' in goal file is not written in the correct format");
        
        actasp::AspFluent fluent(fl);

        bwi_kr_execution::AspFluent fluentMsg;
        fluentMsg.name = fluent.getName();
        vector<string> parameters = fluent.getParameters();
        for (vector<string>::iterator it = parameters.begin(); it != parameters.end(); ++it) {
            fluentMsg.variables.push_back(*it);
        }
        fluentMsg.timeStep = fluent.getTimeStep();

        rule.body.push_back(fluentMsg);

        fluent_begin = s.find_first_not_of(", .", fluent_end + 1);
        fluent_end = s.find(")", fluent_begin);
    }
    return rule;
}

PetriNetPlans::PnpExecutable* TaskInstantiator::createAction(const std::string& name) {

    bwi_kr_execution::ExecutePlanGoal goal;
    stringstream goal_s;
    goal_s << name << "\n";
    string line;
    string fileName = ros::package::getPath("bwi_tasks_pnp")+"/task_types/"+ name + ".txt";
    ifstream goalFile(fileName.c_str());

    if ((goalFile) && (goalFile.is_open())) {

        while (getline(goalFile,line)) {
            goal.aspGoal.push_back(parseAspRule(line));
            goal_s << line;
        }
        
        ROS_INFO_STREAM(goal_s.str());
        goalFile.close();
        
        return new Task(goal, client);   
    }
    else return NULL; 

}

