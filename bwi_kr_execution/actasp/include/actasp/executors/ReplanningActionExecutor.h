#ifndef actasp_ReplanningActionExecutor_h__guard
#define actasp_ReplanningActionExecutor_h__guard


#include <actasp/ActionExecutor.h>

#include <stdexcept>
#include <list>

namespace actasp {

class AspKR;
class Planner;
class Action;

class ReplanningActionExecutor : public ActionExecutor {

public:
	
	ReplanningActionExecutor(actasp::AspKR* reasoner, actasp::Planner *planner) throw (std::invalid_argument);
	
	void setGoal(const AspRule& goalRule) throw();
	void setGoal(const std::vector<actasp::AspRule>& goalRules) throw();

	bool goalReached() const throw() {
		return isGoalReached;
	}
	
	bool failed() const throw() {
		return hasFailed;
	}

	void executeActionStep();
	

private:
	std::vector<actasp::AspRule> goalRules;
	bool isGoalReached;
	bool hasFailed;
	bool actionRunning;
	
	std::list<Action *> plan;
	
	void clearPlan() throw();
	
	AspKR* kr;
	Planner *planner;


};


}
#endif
