
#ifndef actasp_AnyPlan_h__guard
#define actasp_AnyPlan_h__guard

#include <actasp/Planner.h>


namespace actasp {
	
class MultiPlanner;

class AnyPlan : public Planner {
public:
	
	//doesn't own
	AnyPlan(actasp::MultiPlanner *actualPlanner);
	
	std::list<Action *> computePlan(const std::vector<actasp::AspRule>& goal) throw (std::logic_error);
private:
	MultiPlanner *actualPlanner;
	
};

}


#endif
