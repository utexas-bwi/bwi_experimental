#ifndef actasp_Planner_h__guard
#define actasp_Planner_h__guard

#include <actasp/AspRule.h>

#include <vector>
#include <list>
#include <stdexcept>

namespace actasp {

class Action;

struct Planner {

	virtual std::list<Action *> computePlan(const std::vector<actasp::AspRule>& goal) throw (std::logic_error) =0;

	virtual ~Planner() {}
};
	
}
#endif
