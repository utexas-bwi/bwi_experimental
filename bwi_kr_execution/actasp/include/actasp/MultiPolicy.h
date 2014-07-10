#ifndef actasp_MultiPolicy_h__guard
#define actasp_MultiPolicy_h__guard

#include <actasp/AnswerSet.h>

#include <vector>
#include <map>
#include <stdexcept>

namespace actasp {

class Action;

class MultiPolicy {
public:
	std::vector<Action *> actions(const AnswerSet& state)const throw();
	
	void merge(const AnswerSet& plan,const std::map<std::string, Action * >&actionMap) throw(std::logic_error);
	
	bool empty()const throw();
	
	~MultiPolicy();
	
private:
	std::map<AnswerSet, std::vector<Action *> > policy;
	
};
	
}

#endif
