#ifndef bwi_actexec_LogicalNavigation_h__guard
#define bwi_actexec_LogicalNavigation_h__guard

#include "actasp/Action.h"

namespace bwi_krexec {

	
class LogicalNavigation : public actasp::Action {
public:
	
	explicit LogicalNavigation(const std::string &name,const std::vector<std::string>& parameters = std::vector<std::string>());
	
	int paramNumber() const {return parameters.size();}
	
	std::string getName() const {return name;}
	
	void run();
	
	bool hasFinished() const {return done;}
	
	Action *cloneAndInit(const actasp::AspFluent & fluent) const;
	
	Action *clone() const {return new LogicalNavigation(*this);}
	
private:
	
	virtual std::vector<std::string> getParameters() const {return parameters;}
	
	std::string name;
	std::vector<std::string> parameters;
	bool done;

};	
}

#endif
