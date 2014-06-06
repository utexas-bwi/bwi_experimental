

#include "actions/Action.h"
#include "actions/ActionFactory.h"


#include <bwi_kr/ComputeAnswerSet.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>

#include <list>

using namespace bwi_actexec;
using namespace std;



std::list<Action *> computePlan(const std::string& ,unsigned int);
bool checkPlan(const std::list<Action *> & plan, const std::string& goalSpecification);

void generateQuery(const string& goalSpecification, unsigned int n, const string& queryPath);

int main(int argc, char** argv) {

	ros::init(argc, argv, "bwi_action_executor");
	ros::NodeHandle n;

	ros::Rate loop(1.);
	

	//forever

	//TODO wait for a goal


	string goal = ":- not at(o3_512,n).";
	std::list<Action *> plan = computePlan(goal, 10);
	
	cerr << "plan size " << plan.size() <<  endl;
	
	if(plan.empty())
		throw runtime_error("The plan to achieve " + goal + " is empty!");
	
	
	Action * currentAction = plan.front();
	plan.pop_front();

	while (ros::ok()) {

		ros::spinOnce();

		if (!currentAction->hasFinished()) {
			currentAction->run();
		}
		else {

			bool valid = checkPlan(plan,goal);
			
			//TODO consider plan repair

			delete currentAction;

			if (plan.empty())
				return 0;

			currentAction = plan.front();
			plan.pop_front();

		}
		loop.sleep();
	}

	return 0;
}



std::list<Action *> computePlan(const std::string& goalSpecification, unsigned int max_n) {

	string packagePath = ros::package::getPath("bwi_action_executor") +"/";
	string queryPath = packagePath+"queries/ourquery.asp";

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bwi_kr::ComputeAnswerSet> ("/bwi_kr/compute_answer_set");

	client.waitForExistence();

	bwi_kr::ComputeAnswerSet answerSet;
	
	answerSet.request.queryFile = queryPath;

	for (int i=0; i<max_n && !answerSet.response.answerSet.satisfied ; ++i) {

		generateQuery(goalSpecification, i, queryPath);

		client.call(answerSet);

	}


	vector<bwi_kr::Predicate> &preds = answerSet.response.answerSet.predicates;
	vector<Action *> planVector(preds.size());

	for (int j=0 ; j<preds.size() ; ++j) {
		
		Action *act = ActionFactory::byName(preds[j].name);
		act->init(preds[j].parameters);
		planVector[preds[j].timeStep] = act;
	}

	list<Action *> plan(planVector.begin(),planVector.end());
	
	return plan;
}

bool checkPlan(const std::list<Action *> & plan, const std::string& goalSpecification) {
	string packagePath = ros::package::getPath("bwi_action_executor") +"/";
	string queryPath = packagePath+"queries/ourquery.asp";

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bwi_kr::ComputeAnswerSet> ("/bwi_kr/compute_answer_set");

	client.waitForExistence();

	bwi_kr::ComputeAnswerSet answerSet;
	
	answerSet.request.queryFile = queryPath;
	
	stringstream queryStream;

	list<Action *>::const_iterator planIt = plan.begin();
	
	for(unsigned int timeStep = 0; planIt != plan.end(); ++planIt, ++timeStep) {
		queryStream << (*planIt)->toASP(timeStep) << "." << endl;
	}
	
	queryStream << goalSpecification << endl;

	generateQuery(queryStream.str(), plan.size()+1, queryPath);

	client.call(answerSet);

	return answerSet.response.answerSet.satisfied;

}



void generateQuery(const string& goalSpecification, unsigned int n, const string& queryPath) {
	
	ofstream queryFile(queryPath.c_str());
	
	queryFile << "#const n=" << n << "." << endl;
	
	queryFile << goalSpecification  <<endl;
	
	queryFile << "#hide." << endl;
	
	ActionFactory::ActionMap::const_iterator actIt = ActionFactory::actions().begin();
	for( ; actIt != ActionFactory::actions().end(); ++actIt) {
		
		//the last parameter is always the the step number
		queryFile << "#show " << actIt->second->getName() << "/" << actIt->second->paramNumber() + 1 << "." << endl;
		
	}
	
	queryFile.close();
}