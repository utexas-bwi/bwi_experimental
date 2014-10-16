#include "plan_concurrently.h"

#include "actions/ActionFactory.h"
#include "kr_interface.h"

#include "bwi_kr/AnswerSetMsg.h"

#include "ros/ros.h"

#include <sstream>

using namespace std;

namespace bwi_actexec {
	
Replan::Replan(const std::string& goal, int max_n) :
	goalSpecification(goal),
	max_n(max_n),
	finalPlan(),
	done(false),
	copied(false) {}
	
Replan::~Replan() {
	if(!copied)
		for_each(finalPlan.begin(), finalPlan.end(), DeleteAction());
}


void Replan::operator()() {
	bwi_kr::AnswerSetMsg answerSet;

	for (int i=0; i<max_n && !answerSet.satisfied ; ++i) {

		stringstream goal;
		goal << goalSpecification << endl;
		goal << "#hide." << endl;

		ActionFactory::ActionMap::const_iterator actIt = ActionFactory::actions().begin();
		for (; actIt != ActionFactory::actions().end(); ++actIt) {
			//the last parameter is always the the step number
			goal << "#show " << actIt->second->getName() << "/" << actIt->second->paramNumber() + 1 << "." << endl;
		}

		answerSet = kr_query(goal.str(),i,"planQuery.asp");

	}

	vector<bwi_kr::Predicate> &preds = answerSet.predicates;

	vector<Action *> planVector(preds.size());
	for (int j=0 ; j<preds.size() ; ++j) {

		Action *act = ActionFactory::byName(preds[j].name);
		act->init(preds[j].parameters);
		planVector.at(preds[j].timeStep) = act;
	}

	finalPlan.insert(finalPlan.begin(),planVector.begin(),planVector.end());
	done = true;
}

Repair::Repair(const std::list<Action *>& plan,const std::string& goal, unsigned int max_changes) :
				plan(),
				goalSpecification(goal),
				max_changes(max_changes),
				finalPlan(),
				done(false) {
					//make a deep copy
					transform(plan.begin(),plan.end(),back_inserter(this->plan), CloneAction());
				}
Repair::~Repair() {
	for_each(plan.begin(), plan.end(), DeleteAction());
	if(!copied)
		for_each(finalPlan.begin(), finalPlan.end(), DeleteAction());
}

void Repair::operator()() {
	
	ROS_DEBUG_STREAM("repairing..." << "maximum number of changes is " << max_changes);

	bwi_kr::AnswerSetMsg answerSet;

	for (int i=1; (i<=max_changes) ; ++i) {

		int insert_N = i; //number of actions inserted to old plan
		int delete_N = 0; //number of actions deleted from old plan

		std::list<Action *> reusedPlan(plan.begin(), plan.end());

		while (insert_N >= 0) {
			ROS_DEBUG_STREAM("insert " << insert_N << " remove " << delete_N);

			stringstream queryStream;

			list<Action *>::const_iterator planIt = reusedPlan.begin();

			for (unsigned int timeStep = insert_N; planIt != reusedPlan.end(); ++planIt, ++timeStep) {
				queryStream << (*planIt)->toASP(timeStep) << "." << endl;
			}

			queryStream << goalSpecification << endl;
			ROS_DEBUG_STREAM(queryStream.str());
			queryStream << "#hide." << endl;

			ActionFactory::ActionMap::const_iterator actIt = ActionFactory::actions().begin();
			for (; actIt != ActionFactory::actions().end(); ++actIt) {
				//the last parameter is always the the step number ???
				queryStream << "#show " << actIt->second->getName() << "/" << actIt->second->paramNumber() + 1 << "." << endl;
			}

			bwi_kr::AnswerSetMsg answerSet = kr_query(queryStream.str(),reusedPlan.size()+insert_N, "repairPlan.asp");
			if (answerSet.satisfied) {
				ROS_DEBUG("satisfied");
				vector<bwi_kr::Predicate> &preds = answerSet.predicates;
				vector<Action *> planVector(preds.size());

				for (int j=0 ; j<preds.size() ; ++j) {

					ROS_DEBUG_STREAM("predicate timestep: " << preds[j].timeStep);

					Action *act = ActionFactory::byName(preds[j].name);
					act->init(preds[j].parameters);
					planVector.at(preds[j].timeStep) = act;
				}

				finalPlan.insert(finalPlan.begin(),planVector.begin(),planVector.end());
				done = true;
				return;

			}

			insert_N--;
			delete_N++;

			if (!reusedPlan.empty())
				reusedPlan.pop_front();
		}
	}
	
	done = true; //will return the empty list
}


}