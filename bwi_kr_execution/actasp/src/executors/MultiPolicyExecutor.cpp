#include <actasp/executors/MultiPolicyExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/MultiPlanner.h>
#include <actasp/AspRule.h>
#include <actasp/ActionSelector.h>
#include <actasp/Action.h>


#include <algorithm>

using namespace std;

namespace actasp {

void  MultiPolicyExecutor::setGoal(const std::vector<actasp::AspRule>& goalRules) throw() {

	this->goalRules = goalRules;

	isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

	if (!isGoalReached)
		policy = planner->computePolicy(goalRules,suboptimality);

	hasFailed = policy.empty();
	delete active;
	active = NULL;

}

bool MultiPolicyExecutor::goalReached() const throw() {
	return isGoalReached;
}
bool MultiPolicyExecutor::failed() const throw() {
	return hasFailed;
}



void MultiPolicyExecutor::executeActionStep() {
	if (isGoalReached || hasFailed)
		return;
	
	if(active != NULL && !active->hasFinished())
		active->run();
	else {
		AnswerSet currentState = kr->currentStateQuery(vector<AspRule>());

		//let the chooser know that the action has finished
		//and what it led to
		if(active != NULL)
			selector->actionOutcome(active,currentState);
		
		isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();
		
		if(isGoalReached) //well done!
			return;
		
		//choose the next action
		vector<Action*> options = policy.actions(currentState);
		
		if(options.empty()) {
			//there's no action for this state, computing more plans
			//TODO using only optimal plans for now, consider using suboptimal ones too
			MultiPolicy otherPolicy = planner->computePolicy(goalRules,1);
			policy.merge(otherPolicy);
			
			options = policy.actions(currentState);
			if(options.empty()) {//no actions available from here!
				hasFailed = true;
				return;
			}
		}		
		vector<Action *>::iterator chosen = selector->choose(currentState,options);
		
		delete active;
		active = *chosen;
		
		options.erase(chosen);
		for_each(options.begin(),options.end(),ActionDeleter());
	}
	
}

}
