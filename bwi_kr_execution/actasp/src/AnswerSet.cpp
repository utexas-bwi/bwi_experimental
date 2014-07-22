#include <actasp/AnswerSet.h>

#include <actasp/Action.h>

#include <algorithm>

using namespace std;

namespace actasp {

AnswerSet::AnswerSet(bool satisfied,const std::vector<actasp::AspFluent>& fluents) throw () :
	satisfied(satisfied),
	fluents(fluents)  {

	sort((this->fluents).begin(),(this->fluents).end());
}

bool AnswerSet::isSatisfied() const throw() {
	return satisfied;
}

bool AnswerSet::contains(const actasp::AspFluent& fluent) const throw() {
	return binary_search(fluents.begin(),fluents.end(),fluent);
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

	vector<AspFluent>::const_iterator fluentIt = fluents.begin();

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

std::vector<actasp::AspFluent> AnswerSet::getFluentsAtTime(unsigned int timeStep) const throw() {
	
	//create a fake fluent which would be the first one with that time stemp (fluents are ordered)
	AspFluent fake("-",vector<string>(),timeStep);
	
	std::vector<actasp::AspFluent>::const_iterator element =  lower_bound(fluents.begin(),fluents.end(),fake);
	
	vector<AspFluent> target;
	for(; element != fluents.end() && element->getTimeStep() == timeStep; ++element)
		target.push_back(*element);
	
	return target;
}

bool AnswerSet::operator<(const AnswerSet &other) const throw() {
	if(this->fluents.size() != other.fluents.size())
		return this->fluents.size() < other.fluents.size();
	
	//they have the same number of fluents
	vector<AspFluent>::const_iterator thisV = this->fluents.begin();
	vector<AspFluent>::const_iterator otherV = other.fluents.begin();
	
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