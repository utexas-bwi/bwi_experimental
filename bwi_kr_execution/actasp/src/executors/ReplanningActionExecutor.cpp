

#include <actasp/executors/ReplanningActionExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/AnswerSet.h>
#include <actasp/Planner.h>
#include <actasp/Action.h>
#include <actasp/action_utils.h>

#include <list>
#include <algorithm>
#include <iterator>

using namespace std;

namespace actasp {

ReplanningActionExecutor::ReplanningActionExecutor(actasp::AspKR* reasoner,
    actasp::Planner *planner,
    const std::map<std::string, Action * > &actionMap
                                                  ) throw (std::invalid_argument) :
  goalRules(),
  isGoalReached(true),
  hasFailed(false),
  actionMap(),
  plan(),
  kr(reasoner),
  planner(planner) {
  if (reasoner == NULL)
    throw invalid_argument("ReplanningActionExecutor: reasoner is NULL");

  if (planner == NULL)
    throw invalid_argument("ReplanningActionExecutor: planner is NULL");

  transform(actionMap.begin(),actionMap.end(),inserter(this->actionMap,this->actionMap.begin()),ActionMapDeepCopy());
}

ReplanningActionExecutor::~ReplanningActionExecutor() {
  for_each(actionMap.begin(),actionMap.end(),ActionMapDelete());
}


void ReplanningActionExecutor::setGoal(const std::vector<actasp::AspRule>& goalRules) throw() {
  this->goalRules = goalRules;

  isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

  if (!isGoalReached)
    plan = planner->computePlan(goalRules).instantiateActions(actionMap);

  hasFailed = plan.empty();
}


void ReplanningActionExecutor::executeActionStep() {

  if (isGoalReached || hasFailed)
    return;


  Action *current = plan.front();

  current->run();

  if (current->hasFinished()) {
    //destroy the action and pop a new one
    delete current;
    plan.pop_front();

    if (plan.empty() || !kr->isPlanValid(plan,goalRules)) {
      
      //if not valid, replan
      clearPlan();

      isGoalReached = kr->currentStateQuery(goalRules).isSatisfied();

      if (!isGoalReached)
        plan = planner->computePlan(goalRules).instantiateActions(actionMap);

      hasFailed = !isGoalReached && plan.empty();

      return;

    }

  }
}

void ReplanningActionExecutor::clearPlan() throw() {
  std::list<Action *>::iterator actIt = plan.begin();

  for (; actIt != plan.end(); ++actIt)
    delete *actIt;

  plan.clear();
}


}
