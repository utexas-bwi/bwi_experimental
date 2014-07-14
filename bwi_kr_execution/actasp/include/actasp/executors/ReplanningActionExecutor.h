#ifndef actasp_ReplanningActionExecutor_h__guard
#define actasp_ReplanningActionExecutor_h__guard


#include <actasp/ActionExecutor.h>

#include <stdexcept>
#include <list>
#include <map>

namespace actasp {

class AspKR;
class Planner;
class Action;

class ReplanningActionExecutor : public ActionExecutor {

public:
	
	ReplanningActionExecutor(actasp::AspKR* reasoner, 
							 actasp::Planner *planner,
							 const std::map<std::string, Action * > &actionMap
							) throw (std::invalid_argument);
	
	void setGoal(const std::vector<actasp::AspRule>& goalRules) throw();

	bool goalReached() const throw() {
		return isGoalReached;
	}
	
	bool failed() const throw() {
		return hasFailed;
	}

	void executeActionStep();
	
	~ReplanningActionExecutor();
	

private:
	std::vector<actasp::AspRule> goalRules;
	bool isGoalReached;
	bool hasFailed;
	bool actionRunning;
	std::map<std::string, Action * > actionMap;
	
	std::list<Action *> plan;
	
	void clearPlan() throw();
	
	AspKR* kr;
	Planner *planner;


};


}
#endif
