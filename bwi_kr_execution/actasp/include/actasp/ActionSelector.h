#ifndef actasp_ActionSelector_h__guard
#define actasp_ActionSelector_h__guard

#include <actasp/AnswerSet.h>

#include <vector>

namespace actasp {
	
class Action;

struct ActionSelector {

	virtual std::vector<Action*>::iterator choose(const actasp::AnswerSet& currentState,std::vector<Action*> options) = 0;
	
	virtual void actionOutcome(Action *last,const actasp::AnswerSet& nextState) = 0;

	virtual ~ActionSelector() {}
};
}

#endif
