#include <actasp/planners/AnyPlan.h>

#include <actasp/MultiPlanner.h>
#include <actasp/Action.h>

#include <cstdlib>

using namespace std;

namespace actasp {

AnyPlan::AnyPlan(actasp::MultiPlanner *actualPlanner) : actualPlanner(actualPlanner) {}

std::list<Action *> AnyPlan::computePlan(const std::vector<actasp::AspRule>& goal) throw (std::logic_error) {

	vector< list <Action *> > allPlans = actualPlanner->computeAllPlans(goal,1);

	if (allPlans.empty())
		return list<Action *>();

	//pick one plan and return it, destroy the others

	int picked =rand() % allPlans.size();

	for (int i=0, size = allPlans.size(); i<size; ++i) {
		if (i != picked) {
			list<Action *>::iterator onePlanIt = allPlans[i].begin();
			for (; onePlanIt != allPlans[i].end(); ++onePlanIt)
				delete *onePlanIt;
		}
	}

	return allPlans[picked];

}

}
