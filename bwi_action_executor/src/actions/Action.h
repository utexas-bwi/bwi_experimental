#ifndef bwi_actexec_Action_h__guard
#define bwi_actexec_Action_h__guard

#include <string>
#include <vector>

namespace bwi_actexec {
	
	
struct Action {	
	
	virtual void init(const std::vector<std::string>& paramters) {}
	
	virtual int paramNumber() const = 0;
	
	virtual std::string getName() const = 0;
	
	virtual void run() = 0;
	
	virtual bool hasFinished() const = 0;
	
	virtual Action *clone() const =0;
	
	virtual std::string toASP(unsigned int timeStep) const  = 0;
	
virtual ~Action() {}
};
	
	
}
#endif