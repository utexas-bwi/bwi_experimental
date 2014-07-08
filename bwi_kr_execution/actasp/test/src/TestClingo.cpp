#include "unittest++/UnitTest++.h"

#include <actasp/reasoners/Clingo.h>
#include <actasp/Action.h>
#include <actasp/AspRule.h>

#include "SimpleAction.h"

#include <map>
#include <string>
#include <list>
#include <vector>
#include <stdexcept>


struct ClingoFixture {
	ClingoFixture() {

		std::map<std::string, actasp::Action *> actionMap;
		actionMap.insert(std::make_pair(std::string("north"), new SimpleAction("north")));
		actionMap.insert(std::make_pair(std::string("east"), new SimpleAction("east")));
		//use the fact this action is missing to test an exception
		//actionMap.insert(std::make_pair(std::string("south"), new SimpleAction("south")));
		actionMap.insert(std::make_pair(std::string("west"), new SimpleAction("west")));

		clingo = new actasp::Clingo(36, "../test/queries/", "../test/griddomain/",actionMap);
		
		//set initial state to 2,0
		std::vector<actasp::AspFluent> initialObs;
		initialObs.push_back(actasp::AspFluent("pos(2,0,0)"));
		CHECK(clingo->updateFluents(initialObs));

	}
	~ClingoFixture() {
		delete clingo;
	}

	actasp::Clingo *clingo;

};

TEST_FIXTURE(ClingoFixture, updateCurrentStateTest) {
	
	//I'm initially in 2,1
	std::vector<actasp::AspFluent> initialObs;
	initialObs.push_back(actasp::AspFluent("pos(2,1,0)"));
	
	CHECK(clingo->updateFluents(initialObs));

	//I'm initially not in 2,0
	std::vector<actasp::AspRule> stateTest1(1);
	stateTest1[0].body.push_back(actasp::AspFluent("not pos(2,0,0)"));

	CHECK(!(clingo->currentStateQuery(stateTest1).isSatisfied()));
	
	std::vector<actasp::AspFluent> observations;
	observations.push_back(actasp::AspFluent("pos(2,0,0)"));
	
	CHECK(clingo->updateFluents(observations));
	
	CHECK(clingo->currentStateQuery(stateTest1).isSatisfied());
}


TEST_FIXTURE(ClingoFixture, planningTest) {

	std::vector<actasp::AspRule> goalFormula;
	goalFormula.push_back(actasp::AspFluent("not pos(3,4,n)"));

	std::list<actasp::Action *> correctPlan;
	correctPlan.push_back(new SimpleAction("north"));
	correctPlan.push_back(new SimpleAction("north"));
	correctPlan.push_back(new SimpleAction("north"));
	correctPlan.push_back(new SimpleAction("north"));
	correctPlan.push_back(new SimpleAction("east"));

 	std::list<actasp::Action *> plan = clingo->computePlan(goalFormula);

	bool correct = plan.size() == correctPlan.size();

	std::list<actasp::Action *>::iterator planIt= plan.begin();
	std::list<actasp::Action *>::iterator corrIt= correctPlan.begin();

	for (; planIt != plan.end(); ++planIt, ++corrIt) {
		correct = correct && (((*planIt)->getName()) == ((*corrIt)->getName()));
		delete(*planIt);
		delete(*corrIt);
	}

	CHECK(correct);
	
	//set an initial state and a goal that requires the missing action south
	std::vector<actasp::AspFluent> initialObs;
	initialObs.push_back(actasp::AspFluent("pos(3,4,0)"));
	CHECK(clingo->updateFluents(initialObs));
	
	goalFormula.clear();
	goalFormula.push_back(actasp::AspFluent("not pos(3,3,n)"));
	
	CHECK_THROW(clingo->computePlan(goalFormula), std::logic_error);
	
	
}

TEST_FIXTURE(ClingoFixture, emptyPlanTest) {

	std::vector<actasp::AspRule> goalFormula;
	goalFormula.push_back(actasp::AspFluent("not pos(2,0,n)"));

	std::list<actasp::Action *> plan = clingo->computePlan(goalFormula);

	CHECK(plan.empty());
}

TEST_FIXTURE(ClingoFixture, planValidityTest) {
	

	std::vector<actasp::AspRule> goalFormula;
	goalFormula.push_back(actasp::AspFluent("not pos(3,4,n)"));

	std::list<actasp::Action *> plan = clingo->computePlan(goalFormula);

	CHECK(clingo->isPlanValid(plan,goalFormula));

	std::vector<actasp::AspRule> wrongGoal;
	wrongGoal.push_back(actasp::AspFluent("not pos(1,4,n)"));

	CHECK(!(clingo->isPlanValid(plan,wrongGoal)));
	
	std::list<actasp::Action *>::iterator planIt = plan.begin();
	for(; planIt != plan.end(); ++planIt)
		delete *planIt;
}

TEST_FIXTURE(ClingoFixture, currentStateTest) {

	//I am in 2,0
	//should be satisfied, it's the initial position
	std::vector<actasp::AspRule> stateTest1(1);
	stateTest1[0].head.push_back(actasp::AspFluent("pos(2,0,0)"));

	CHECK(clingo->currentStateQuery(stateTest1).isSatisfied());
	
	//I am in 2,1 (note head)
	//should not be satisfied, I am in 2,0 and cannot be in two places
	//at the same time
	std::vector<actasp::AspRule> stateTest2(1);
	stateTest2[0].head.push_back(actasp::AspFluent("pos(2,1,0)"));
	
	CHECK(!(clingo->currentStateQuery(stateTest2).isSatisfied()));
	
	//It is impossible that I am not in 2,1 (note body)
	//should not be satisfied, I am in 2,0 and cannot prove I am in 2,1
	std::vector<actasp::AspRule> stateTest3(1);
	stateTest3[0].body.push_back(actasp::AspFluent("not pos(2,1,0)"));
	
	CHECK(!(clingo->currentStateQuery(stateTest3).isSatisfied()));
}

TEST_FIXTURE(ClingoFixture, allPlansTest) {
	
	std::vector<actasp::AspRule> goalFormula;
	goalFormula.push_back(actasp::AspFluent("not pos(4,2,n)"));
	
	actasp::MultiPolicy policy = clingo->computePolicy(goalFormula,1);
	
	std::vector<actasp::Action *> actions = policy.actions(clingo->currentStateQuery(std::vector<actasp::AspRule>())); 

	CHECK_EQUAL(2,actions.size());
	
	CHECK(	(actions[0]->getName() == "north") && (actions[1]->getName() == "east") || 
			(actions[0]->getName() == "east") && (actions[1]->getName() == "north"));
	
	std::vector<actasp::AspFluent> anotherState;
	anotherState.push_back(actasp::AspFluent("pos(4,1,0)"));
	
	CHECK(clingo->updateFluents(anotherState));
	
	std::vector<actasp::Action *>::iterator actIt = actions.begin();
	for(; actIt != actions.end(); ++actIt)
		delete *actIt;
	
	actions = policy.actions(clingo->currentStateQuery(std::vector<actasp::AspRule>()));
	
	
	CHECK_EQUAL(1,actions.size());
	
	CHECK(actions[0]->getName() == "north");
	
	delete actions[0];
}
