
#include "unittest++/UnitTest++.h"

#include "SimpleAction.h"

#include <actasp/AnswerSet.h>
#include <actasp/AspFluent.h>


#include <vector>
#include <algorithm>

using namespace std;
using namespace actasp;

TEST(AnswerSetContains) {

	std::vector<actasp::AspFluent> fluents;
	
	fluents.push_back(AspFluent("A(alice,bob,0)"));
	fluents.push_back(AspFluent("B(alice,bob,0)"));
	fluents.push_back(AspFluent("-A(carol,bob,0)"));
	
	AnswerSet answer(true,fluents);
	
	CHECK(answer.contains(AspFluent("B(alice,bob,0)")));
	CHECK(!answer.contains(AspFluent("B(alice,bob,1)")));
	CHECK(answer.contains(AspFluent("-A(carol,bob,0)")));
	CHECK(!answer.contains(AspFluent("A(carol,bob,0)")));
	
}

TEST(AnswerSetInstantiateActions) {
	
	std::map<std::string, actasp::Action *> actionMap;
	actionMap.insert(std::make_pair(std::string("north"), new SimpleAction("north")));
	actionMap.insert(std::make_pair(std::string("east"), new SimpleAction("east")));
	
	std::vector<actasp::AspFluent> fluents;
	fluents.push_back(AspFluent("north(param,0)"));
	fluents.push_back(AspFluent("east(param,1)"));
	
	AnswerSet answer(true,fluents);
	
	list<Action*> plan = answer.instantiateActions(actionMap);
	
	bool correct = plan.front()->getName() == "north";
	delete plan.front();
	plan.pop_front();
	correct = correct && plan.front()->getName() == "east";
	delete plan.front();
	plan.pop_front();
	
	correct = correct && plan.empty();
	
	CHECK(correct);
	
	//exceptional cases
	//there's a hole in the plan
	fluents.push_back(AspFluent("east(param,3)"));
	answer = AnswerSet(true,fluents);
	
	CHECK_THROW(answer.instantiateActions(actionMap), logic_error);
	
	std::map<std::string, actasp::Action *>::iterator actMapIt = actionMap.begin();
	for(; actMapIt != actionMap.end(); ++actMapIt)
		delete (actMapIt->second);
}

TEST(AnswerSetGetFluentsAtTime) {
	std::vector<actasp::AspFluent> fluents;
	fluents.push_back(AspFluent("north(param,0)"));
	fluents.push_back(AspFluent("east(param,1)"));
	fluents.push_back(AspFluent("pos(param,1)"));
	fluents.push_back(AspFluent("-pos(param2,1)"));
	fluents.push_back(AspFluent("east(param,2)"));
	
	sort(fluents.begin(),fluents.end());
	
	AnswerSet answer(true,fluents);
	
	std::vector<actasp::AspFluent> selected = answer.getFluentsAtTime(1);
	
	CHECK_EQUAL(3, selected.size());
	
	CHECK(std::equal(selected.begin(),selected.end(),++(fluents.begin())));
	
	CHECK_EQUAL(0, answer.getFluentsAtTime(4).size());
	
}

