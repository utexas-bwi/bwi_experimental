#ifndef actasp_MultiPolicyExecutor_h__guard
#define actasp_MultiPolicyExecutor_h__guard

#include <actasp/ActionExecutor.h>
#include <actasp/MultiPolicy.h>
#include <actasp/AspRule.h>

#include <vector>

namespace actasp {
	
class AspKR;
class MultiPlanner;
class ActionSelector;

class MultiPolicyExecutor : public ActionExecutor {
public:
	void setGoal(const std::vector<actasp::AspRule>& goalRules) throw() = 0;

	bool goalReached() const throw() =0;
	bool failed() const throw() = 0;

	void executeActionStep() = 0;
private:
	bool isGoalReached;
	bool hasFailed;
	std::vector<actasp::AspRule> goalRules;
	AspKR* kr;
	MultiPlanner *planner;
	MultiPolicy policy;
	double suboptimality;
	Action *active;
	ActionSelector *selector;
	
};

}

#endif 
