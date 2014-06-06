#include "kr_interface.h"

#include <bwi_kr/ComputeAnswerSet.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>

using namespace std;

bwi_kr::AnswerSetMsg  kr_query(const std::string& query, unsigned int lastState, const std::string queryFileName) {
	
	string packagePath = ros::package::getPath("bwi_action_executor") +"/";
	string queryPath = packagePath+"queries/" + queryFileName;
	
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bwi_kr::ComputeAnswerSet> ("/bwi_kr/compute_answer_set");

	client.waitForExistence();

	bwi_kr::ComputeAnswerSet answerSetParameters;
	
	answerSetParameters.request.queryFile = queryPath;
	
	ofstream queryFile(queryPath.c_str());
	queryFile << "#const n=" << lastState<< "." << endl;
	queryFile << query  <<endl;
	queryFile.close();
	
	client.call(answerSetParameters);

	return answerSetParameters.response.answerSet;
}