
// -*- mode: C++ -*-
// -*- c-file-style: bsd -*-

///
//   bwi_action_executor -- main program
//

#include "action_utils.h"

#include "actions/Action.h"
#include "actions/ActionFactory.h"
#include "actions/LogicalNavigation.h"

#include "kr_interface.h"
#include "plan_concurrently.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/bind.hpp>
#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>

#include <list>
#include <algorithm>
#include <iterator>

#define SEQUENTIAL 0

using namespace bwi_actexec;
using namespace std;

const unsigned int MAX_N = 50;          // maximum number of plan steps

std::list<Action *> computePlan(const std::string& ,unsigned int);
bool checkPlan(const std::list<Action *> & plan, const std::string& goalSpecification);
bool repairOrReplan(const std::list<Action *> & plan, const std::string& goalSpecification, unsigned int max_changes, std::list<Action *> & newPlan);

///  Delete all the actions in a plan.

enum State {
	pickAction,
	executeAction,
	verifyPlan,
	recomputePlan,
	waitForGoal
};

int main(int argc, char** argv) {

	ros::init(argc, argv, "bwi_action_executor");
	ros::NodeHandle n;

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	}

	ros::Rate loop(10);

	//noop updates the fluents with the current position
	LogicalNavigation setInitialState("noop");
	setInitialState.run();

	string goal = ":- not at(l3_500,n).";

	std::list<Action *> plan = computePlan(goal, MAX_N);
	
	stringstream initialPlanString;
	transform(plan.begin(),plan.end(),ostream_iterator<std::string>(initialPlanString),ActionToAsp());

	ROS_DEBUG_STREAM("Plan Length is: " << plan.size());
	ROS_DEBUG_STREAM("The list of all the actions is: " << initialPlanString.str());


	if (plan.empty())
		throw runtime_error("The plan to achieve " + goal + " is empty!");

	State currentState = pickAction;
	unsigned int executed = 0;


	while (ros::ok()) {
		
		Action * currentAction;
		
		switch (currentState) {
			
			case pickAction:
				if(plan.empty())
					currentState = waitForGoal;
				else {
					currentAction = plan.front();
					plan.pop_front();
					currentState = executeAction;
				}
			break;
			
			case executeAction:
				
				if(!currentAction->hasFinished())
					currentAction->run();
				else {
					delete currentAction;
					currentState = verifyPlan;
				}
				
			break;
			
			case verifyPlan:
				
				ROS_DEBUG("Forward Projecting Plan to Check Validity...");

				if(checkPlan(plan,goal)) {
					ROS_DEBUG("Valid.");
					currentState = pickAction;
				} else {
					ROS_DEBUG("Invalid.");
					currentState = recomputePlan;
				}
				
			break;
			
			case recomputePlan:
			{
				int max_changes = min((MAX_N-executed-plan.size()), plan.size());

				list<Action *> newPlan;
				bool done = repairOrReplan(plan, goal, max_changes,newPlan);
				
				if(done) {
					if(newPlan.empty())
						currentState = waitForGoal;
					else {
						for_each(plan.begin(),plan.end(),DeleteAction());
						plan = newPlan;
						stringstream newPlanString;
						transform(plan.begin(),plan.end(),ostream_iterator<std::string>(newPlanString),ActionToAsp());

						ROS_DEBUG_STREAM("Plan Length is: " << plan.size());
						ROS_DEBUG_STREAM("The list of all the actions is: " << newPlanString.str());
						executed = 0;
						currentState = pickAction;
					}
				}
			}
			break;
			
			case waitForGoal :
			{
				ROS_DEBUG("No more actions to execute, waiting for a new goal");
			}
			break;
			
			
		}

		ros::spinOnce();
		
		loop.sleep();
	}

	return 0;
}


/// Compute a new plan to get from current state to goal.
//
//  @param goalSpecification string representing the goal state.
//  @param max_n maximum number of steps allowed in the plan.
//  @return list of pointer to actions for executing the plan.
//
//  @note: This runs in parallel with the main thread.  It MUST not
//         modify any variables used there.
//
std::list<Action *> computePlan(const std::string& goalSpecification, unsigned int max_n) {

	Replan replan(goalSpecification,MAX_N);
	replan();
	return replan.computedPlan();
}

bool checkPlan(const std::list<Action *> & plan, const std::string& goalSpecification) {

	stringstream queryStream;

	list<Action *>::const_iterator planIt = plan.begin();

	for (unsigned int timeStep = 0; planIt != plan.end(); ++planIt, ++timeStep) {
		queryStream << (*planIt)->toASP(timeStep) << "." << endl;
	}

	queryStream << goalSpecification << endl;

	bwi_kr::AnswerSetMsg answerSet = kr_query(queryStream.str(),plan.size(), "checkPlan.asp");

	return answerSet.satisfied;

}

vector<Operation *> active;
vector<Operation *> deathrow; //operations waiting to die

bool repairOrReplan(const std::list<Action *> & plan,
                                   const std::string& goal,
                                   unsigned int max_changes,
								   std::list<Action *> & newPlan) {

	ROS_DEBUG("repairing or replanning...");

#if SEQUENTIAL

	Repair repair(plan,goal,max_changes);
	
	repair();
	
	if(repair.computedPlan().empty()) {
		//try replanning
		
		Replan replan(goal,MAX_N);
		replan();
		
		newPlan = replan.computedPlan();
		
	}
	else
		newPlan = repair.computedPlan();
	
	return true;
	

#else
	
	ROS_DEBUG("Cleaning the death row");
	ROS_DEBUG_STREAM("size: " << deathrow.size());
	//first, let's do some cleanup
	vector<Operation *>::iterator dIt = deathrow.begin();
	for(;dIt != deathrow.end(); ++dIt) {
		if( (*dIt)->finished() ) {
			delete *dIt;
			dIt = deathrow.erase(dIt);
			if(dIt == deathrow.end()) break; //not very elegant, but if ++dIt increments end() we are done
		}
	}

	ROS_DEBUG_STREAM("size: " << deathrow.size());
	//if it's the first invocation, we need to create the active operations
	if(active.empty()) {
		
		ROS_DEBUG("Creating new operations");

		active.push_back(new Replan(goal,MAX_N));
		active.push_back(new Repair(plan,goal,max_changes));
		
		
		boost::thread replanThread(boost::ref(*active[0]));
		boost::thread repairThread(boost::ref(*active[1]));
		
		//let them go their own way
		repairThread.detach();
		replanThread.detach();
	}
	
	if(active[0]->finished() || active[1]->finished()) {
		
		if(active[0]->finished())
			newPlan = active[0]->computedPlan();
		else
			newPlan = active[1]->computedPlan();
		
		ROS_DEBUG("Done! Moving into the death row");
		deathrow.insert(deathrow.end(), active.begin(),active.end());
		active.clear();
		return true;
	}
	
	return false; 

#endif	
	
}
