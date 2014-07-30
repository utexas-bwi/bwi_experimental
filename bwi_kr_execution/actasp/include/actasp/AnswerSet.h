#ifndef actasp_AnswerSet_h__guard
#define actasp_AnswerSet_h__guard

#include <actasp/AspFluent.h>

#include <string>
#include <set>
#include <list>
#include <map>
#include <stdexcept>

namespace actasp {

class Action;

class AnswerSet {

public:
	
	explicit AnswerSet(bool satisfied,const std::set<actasp::AspFluent>& fluents = std::set<AspFluent>()) throw();
	
	bool isSatisfied() const throw();
	
	bool contains(const actasp::AspFluent& fluent) const throw();
	
	std::list<Action *> instantiateActions(const std::map<std::string, actasp::Action*> &actionMap) const throw(std::logic_error);
	
	const std::set<actasp::AspFluent>& getFluents() const throw() { return fluents;}
	
	std::set<actasp::AspFluent> getFluentsAtTime(unsigned int timeStep) const throw();
  
  unsigned int maxTimeStep() const throw();
	
	bool operator<(const AnswerSet &other) const throw();
	
private:
	bool satisfied;
	std::set<actasp::AspFluent> fluents;
};
	
}
#endif
