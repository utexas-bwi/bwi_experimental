

#include <actasp/executors/ReplanningActionExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/Planner.h>
#include <actasp/Action.h>

#include <list>

using namespace std;

namespace actasp {

ReplanningActionExecutor::ReplanningActionExecutor(actasp::AspKR* reasoner, actasp::Planner *planner) throw (std::invalid_argument) :
	goalRules(),
	isGoalReached(true),
	hasFailed(false),
	actionRunning(false),
	plan(),
	kr(reasoner),
	planner(planner) {
	if (reasoner == NULL)
		throw invalid_argument("ReplanningActionExecutor: reasoner is NULL");

	if (planner == NULL)
		throw invalid_argument("ReplanningActionExecutor: planner is NULL");
}

void ReplanningActionExecutor::setGoal(const AspRule& goalRule) throw() {
	vector<AspRule> goal;
	goal.push_back(goalRule);
	this->setGoal(goal);
}
void ReplanningActionExecutor::setGoal(const std::vector<actasp::AspRule>& goalRules) throw() {
	this->goalRules = goalRules;

	isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

	if (!isGoalReached)
		plan = planner->computePlan(goalRules);

	hasFailed = plan.empty();
}


void ReplanningActionExecutor::executeActionStep() {

	if (isGoalReached || hasFailed)
		return;


	if (!actionRunning && !kr->isPlanValid(plan,goalRules)) {

		//if not valid, replan
		clearPlan();

		isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

		if (!isGoalReached)
			plan = planner->computePlan(goalRules);

		hasFailed = !isGoalReached && plan.empty();


	}
	
	if (plan.empty()) {
		//run out of actions and didn't reach the goal, trying to replan
		plan = planner->computePlan(goalRules);

		hasFailed = plan.empty(); //if there's no plan we fail
		if (hasFailed)
			return;
	}

	Action *current = plan.front();

	current->run();
	
	actionRunning = true;

	if (current->hasFinished()) {
		//destroy the action and pop a new one
		delete current;
		plan.pop_front();
		actionRunning = false;
	}
}

void ReplanningActionExecutor::clearPlan() throw() {
	std::list<Action *>::iterator actIt = plan.begin();

	for (; actIt != plan.end(); ++actIt)
		delete *actIt;

	plan.clear();
}


}
