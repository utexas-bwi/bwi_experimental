
#include <ros/ros.h>
#include <ros/package.h>

#include <sstream>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <bwi_kr/AnswerSet.h>
#include <bwi_kr/ComputeAnswerSet.h>

using namespace std;
using namespace bwi_kr;

bool computeAnswerSet(bwi_kr::ComputeAnswerSet::Request& req,
						bwi_kr::ComputeAnswerSet::Response &res);

int main(int argc, char **argv) {

	ros::init(argc, argv, "bwi_kr");
	ros::NodeHandle n("~");

	ros::ServiceServer service = n.advertiseService("compute_answer_set", computeAnswerSet);
	ros::spin();
	
	return 0;
}


bool computeAnswerSet(bwi_kr::ComputeAnswerSet::Request& req,
						bwi_kr::ComputeAnswerSet::Response &res) {
	
	stringstream commandLine;
	
	string &queryFile = req.queryFile;
	
	string packagePath = ros::package::getPath("bwi_kr")+"/";
	const string outputFilePath = "/tmp/bwi_kr_query_output.txt";
	
	commandLine << "clingo " << queryFile << " " << packagePath << "domain/*.asp " << " > " << outputFilePath;
	
	system(commandLine.str().c_str());
	
	ifstream outputFile(outputFilePath.c_str());
	
	string line;
	string answerSetContent;
	while(outputFile){
		getline(outputFile, line);
		answerSetContent += line;
		answerSetContent += "\n";
	}

	AnswerSet answer(answerSetContent);
	
	res.answerSet.satisfied = answer.isSatisfied();
	res.answerSet.predicates = answer.getPredicates();

	return true;
	
}