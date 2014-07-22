#ifndef actasp_AnswerSet_h__guard
#define actasp_AnswerSet_h__guard

#include <actasp/AspFluent.h>

#include <string>
#include <vector>
#include <list>
#include <map>
#include <stdexcept>

namespace actasp {

class Action;

class AnswerSet {

public:
	
	explicit AnswerSet(bool satisfied,const std::vector<actasp::AspFluent>& fluents = std::vector<AspFluent>()) throw();
	
	bool isSatisfied() const throw();
	
	bool contains(const actasp::AspFluent& fluent) const throw();
	
	std::list<Action *> instantiateActions(const std::map<std::string, actasp::Action*> &actionMap) const throw(std::logic_error);
	
	const std::vector<actasp::AspFluent>& getFluents() const throw() { return fluents;}
	
	std::vector<actasp::AspFluent> getFluentsAtTime(unsigned int timeStep) const throw();
	
	bool operator<(const AnswerSet &other) const throw();
	
private:
	bool satisfied;
	std::vector<actasp::AspFluent> fluents;
};
	
}
#endif
