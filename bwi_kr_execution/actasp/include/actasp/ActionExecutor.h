#ifndef actasp_ActionExecutor_h__guard
#define actasp_ActionExecutor_h__guard

#include <actasp/AspRule.h>

#include <string>

namespace actasp {

struct ActionExecutor {

	virtual void setGoal(const AspRule& goalRule) throw() = 0;
	virtual void setGoal(const std::vector<actasp::AspRule>& goalRules) throw() = 0;

	virtual bool goalReached() const throw() =0;
	virtual bool failed() const throw() = 0;

	virtual void executeActionStep() = 0;

	virtual ~ActionExecutor() {}
};

}

#endif
