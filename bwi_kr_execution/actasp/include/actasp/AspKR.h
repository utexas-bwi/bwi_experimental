#ifndef actasp_AspKR_h__guard
#define actasp_AspKR_h__guard

#include <actasp/AnswerSet.h>
#include <actasp/MultiPlanner.h>

#include <vector>

namespace actasp {

class Action;

struct AspKR : public actasp::MultiPlanner {

	virtual AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() = 0;
	
	//false if the new fluents are incompatible with the KB
	virtual bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw() = 0;
	
	virtual bool isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw() = 0;
	
	virtual ~AspKR() {}
};
	
}
#endif
