#include <actasp/planners/AnyPlan.h>

#include <actasp/MultiPlanner.h>
#include <actasp/Action.h>

#include <cstdlib>

using namespace std;

namespace actasp {

AnyPlan::AnyPlan(actasp::MultiPlanner *actualPlanner) : actualPlanner(actualPlanner) {}

AnswerSet AnyPlan::computePlan(const std::vector<actasp::AspRule>& goal) throw (std::logic_error) {

	vector< AnswerSet > allPlans = actualPlanner->computeAllPlans(goal,1.2);

	if (allPlans.empty())
		return AnswerSet(false);

	//pick one plan and return it, destroy the others

	int picked =rand() % allPlans.size();

	return allPlans[picked];

}

}
