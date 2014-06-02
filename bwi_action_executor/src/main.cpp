

#include "actions/Action.h"
#include "actions/ActionFactory.h"


#include <bwi_kr/ComputeAnswerSet.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>

using namespace bwi_actexec;
using namespace std;



std::vector<Action *> computePlan(const std::string& ,unsigned int);
vector<string> tokenizeCommas(string input);

int main(int argc, char** argv) {

    ros::init(argc, argv, "bwi_action_executor");
    ros::NodeHandle n;

    ros::Rate loop(1.);

    //forever

    //TODO wait for a goal


    std::vector<Action *> plan = computePlan("at(o3_508,n)", 1);
    vector<Action *>::iterator currentAction = plan.begin();

    while(ros::ok()) {

        ros::spinOnce();

        if(!(*currentAction)->hasFinished())
            (*currentAction)->run();
		else {

            //TODO check if the plan is still valid

            ++currentAction;

            if(currentAction == plan.end())
                return 0;

        }
        loop.sleep();
    }

    return 0;
}



std::vector<Action *> computePlan(const std::string& goalSpecification, unsigned int max_n) {
	
	string packagePath = ros::package::getPath("bwi_action_executor")+"/";
	string queryPath = packagePath+"queries/ourquery.asp";
	
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bwi_kr::ComputeAnswerSet>("/bwi_kr/compute_answer_set");
	
	client.waitForExistence();
	
	bwi_kr::ComputeAnswerSet answerSet;
	answerSet.request.queryFile = queryPath;
	
	for(int i=0; i<max_n && !answerSet.response.answerSet.satisfied ; ++i) {
	
		//generateQuery(goalSpecification, i, queryPath);

		client.call(answerSet);

	}
	
	
	vector<string> &preds = answerSet.response.answerSet.predicates;
	vector<Action *> plan(preds.size());
	
	for(int j=0 ; j<preds.size() ; ++j) {
		string name = preds[j].substr(0,preds[j].find_first_of("("));

		Action *act = ActionFactory::byName(name);
		int start = preds[j].find("(")+1;
		int end = preds[j].find_last_of(",");
		string paramString = preds[j].substr(start,end-start);
				
		vector<string> params = tokenizeCommas(paramString);
		
		
		act->init(params);

		//the last integer in the action is the time step
		start = preds[j].find_last_of(",")+1;
		end = preds[j].find_last_of(")");
		int index = atoi((preds[j].substr(start,end-start)).c_str());
		
		plan[index] = act;
	}

	return plan;
}

vector<string> tokenizeCommas(string input) {
	vector<string> stuff;

 	while(!input.empty()) {

 		int firstComma = std::min(input.find(","), input.length());
 		stuff.push_back( input.substr(0,firstComma));
		
		input = input.substr(firstComma);
	}
	return stuff;
}