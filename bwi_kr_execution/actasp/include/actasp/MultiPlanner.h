
#ifndef actasp_MultiPlanner_h__guard
#define actasp_MultiPlanner_h__guard

#include <actasp/Planner.h>
#include <actasp/AspRule.h>
#include <actasp/MultiPolicy.h>

#include <vector>
#include <list>

namespace actasp {

class Action;

struct MultiPlanner : public actasp::Planner {

virtual std::vector< std::list<Action *> > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) throw (std::logic_error)=0;

virtual MultiPolicy computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) throw (std::logic_error)=0;

virtual ~MultiPlanner(){}

};

}

#endif
