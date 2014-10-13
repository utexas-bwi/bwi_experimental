#include <actasp/AnswerSet.h>

#include <actasp/Action.h>

#include <algorithm>

using namespace std;

namespace actasp {

AnswerSet::AnswerSet(bool satisfied,const std::set<actasp::AspFluent>& fluents) throw () :
	satisfied(satisfied),
	fluents(fluents)  {}

bool AnswerSet::isSatisfied() const throw() {
	return satisfied;
}

bool AnswerSet::contains(const actasp::AspFluent& fluent) const throw() {
	return fluents.find(fluent) != fluents.end();
}

static void clearPlan(std::list<actasp::Action*>& plan) {
	std::list<actasp::Action*>::iterator planIt = plan.begin();
	for(; planIt != plan.end(); ++planIt)
		delete (*planIt);
}

std::list<Action *> AnswerSet::instantiateActions(const std::map<std::string, actasp::Action*> &actionMap) const
									throw (std::logic_error) {

	list<Action *> plan;
	unsigned int maxTimeStep = 0;

	set<AspFluent>::const_iterator fluentIt = fluents.begin();

	for (; fluentIt != fluents.end(); ++fluentIt) {
		
		map<string, Action * >::const_iterator actIt = actionMap.find(fluentIt->getName());
		
		if (actIt != actionMap.end()) {
			plan.push_back(actIt->second->cloneAndInit(*fluentIt));
			maxTimeStep = std::max(maxTimeStep,fluentIt->getTimeStep());
		} 
		//if a fluent is not a known action, just ignore it.
	}
	
	if (maxTimeStep > 0 && maxTimeStep >= plan.size()) {
				clearPlan(plan);
				throw logic_error("AnswerSet: the plan is missing an action for some time step. Check the list of actions shown in the plan query.");
	}

	return plan;
}

std::set<actasp::AspFluent> AnswerSet::getFluentsAtTime(unsigned int timeStep) const throw() {
	
	//create fake fluents which would be the upper and lower bound for the time step
	AspFluent fakeLow("-",vector<string>(),timeStep);
  AspFluent fakeUp("-",vector<string>(),timeStep+1);
	
	std::set<actasp::AspFluent>::const_iterator lb =  fluents.lower_bound(fakeLow);
  std::set<actasp::AspFluent>::const_iterator ub =  fluents.lower_bound(fakeUp);
	
	return set<AspFluent>(lb,ub);
}

unsigned int AnswerSet::maxTimeStep() const throw() {
  return fluents.rbegin()->getTimeStep();
}

bool AnswerSet::operator<(const AnswerSet &other) const throw() {
	if(this->fluents.size() != other.fluents.size())
		return this->fluents.size() < other.fluents.size();
	
	//they have the same number of fluents
	set<AspFluent>::const_iterator thisV = this->fluents.begin();
	set<AspFluent>::const_iterator otherV = other.fluents.begin();
	
	for(; thisV != fluents.end(); ++thisV, ++otherV) {
		//this comparison is costly, so I'm using this unelegant expression to minimize the calls to it.
		if(*thisV < *otherV)
			return true;
		if(*otherV < *thisV)
			return false;
	}
	
	//they are the same
	return false;
}



}