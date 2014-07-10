#include <actasp/MultiPolicy.h>

#include <actasp/Action.h>

#include <algorithm>

using namespace std;

namespace actasp {

struct ActionCloner {
	
	Action * operator()(const Action *other) const throw() {
		return other->clone();
	}
};
	
std::vector<Action*> MultiPolicy::actions(const AnswerSet& state) const throw() {
	
	std::map<AnswerSet, std::vector<Action *> >::const_iterator acts = policy.find(state);
	
	vector<Action *> result;
	
	if(acts != policy.end()) {
		//clone all the actions
		transform(acts->second.begin(), acts->second.end(), back_inserter(result), ActionCloner());
	}
	
	return result;
}

struct CompareActions {
	
	CompareActions(Action *first) : first(first) {}
	
	bool operator()(Action *second) {
		return first->operator==(second);
	}
	
	Action *first;
};

struct SetTimeStepToZero {
	void operator()(AspFluent &fluent) {
		fluent.setTimeStep(0);
	}
		
};

void MultiPolicy::merge(const AnswerSet& plan,const std::map<std::string, Action * >&actionMap) throw(logic_error) {
	
	list<Action *> allActions = plan.instantiateActions(actionMap);

	list<Action *>::const_iterator actIt = allActions.begin();

	for (int timeStep = 0; actIt != allActions.end(); ++actIt, ++timeStep) {
		
		vector<AspFluent> fluents = plan.getFluentsAtTime(timeStep);
		
		//remove the action from there
		
		AspFluent actionFluent((*actIt)->toASP(timeStep));
		vector<AspFluent>::iterator newEnd = remove(fluents.begin(),fluents.end(),actionFluent);
		fluents.erase(newEnd,fluents.end());

		for_each(fluents.begin(),fluents.end(),SetTimeStepToZero());
		
		AnswerSet state(true,fluents);
		
		vector<Action *> &stateActions = policy[state]; //creates an empty vector if not present
		
		if( find_if(stateActions.begin(), stateActions.end(),CompareActions(*actIt)) == stateActions.end() )
			stateActions.push_back((*actIt)->clone());
	}
	
	list<Action *>::iterator eraser = allActions.begin();
	for(; eraser != allActions.end(); ++eraser)
		delete *eraser;

}

bool MultiPolicy::empty() const throw() {
	return policy.empty();
}

MultiPolicy::~MultiPolicy() {
	//delete all actions
	map<AnswerSet, vector<Action *> >::iterator policyIt = policy.begin();
	
	for(; policyIt != policy.end(); ++policyIt) {
		vector<Action *>::iterator actIt = policyIt->second.begin();
		
		for(; actIt != policyIt->second.end(); ++actIt)
			delete (*actIt);
	}
}

}
