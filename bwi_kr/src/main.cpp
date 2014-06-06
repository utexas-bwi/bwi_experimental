
#include <ros/ros.h>
#include <ros/package.h>

#include <sstream>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <bwi_kr/AnswerSet.h>
#include <bwi_kr/ComputeAnswerSet.h>
#include <bwi_kr/ChangeFluent.h>

using namespace std;
using namespace bwi_kr;

const std::string domainName = "domain";
string packagePath;

static void createCurrentState(const std::string& observations);
static void createInitialstate(const std::string& initialFile);


bool computeAnswerSet(bwi_kr::ComputeAnswerSet::Request& req,
						bwi_kr::ComputeAnswerSet::Response &res);

bool changeFluent(bwi_kr::ChangeFluent::Request &req,
						bwi_kr::ChangeFluent::Response &res);


int main(int argc, char **argv) {

	ros::init(argc, argv, "bwi_kr");
	ros::NodeHandle n("~");
	
	packagePath = ros::package::getPath("bwi_kr")+"/";
	
	createInitialstate(packagePath + domainName + "/initial.txt");

	ros::ServiceServer computeAS = n.advertiseService("compute_answer_set", computeAnswerSet);
	ros::ServiceServer changeF = n.advertiseService("change_fluent", changeFluent);
	
	ros::spin();

	return 0;
}

static AnswerSet readAnswerSet(const std::string& filePath) {
	
	ifstream file(filePath.c_str());
	
	string line;
	string answerSetContent;
	while(file){
		getline(file, line);
		answerSetContent += line;
		answerSetContent += "\n";
	}

	return AnswerSet(answerSetContent);
}

bool computeAnswerSet(bwi_kr::ComputeAnswerSet::Request& req,
						bwi_kr::ComputeAnswerSet::Response &res) {
	
	stringstream commandLine;
	
	string &queryFile = req.queryFile;
	
	const string outputFilePath = "/tmp/bwi_kr_query_output.txt";
	
	commandLine << "clingo " << queryFile << " " << packagePath << domainName << "/*.asp " << " > " << outputFilePath;
	
	system(commandLine.str().c_str());

	AnswerSet answer = readAnswerSet(outputFilePath);
	
	res.answerSet.satisfied = answer.isSatisfied();
	res.answerSet.predicates = answer.getPredicates();

	return true;

}

std::string concatenateParameters(const std::vector<std::string>& params) {
	
	stringstream ss;
	
	vector<string>::const_iterator paramIt = params.begin();
		for(; paramIt != params.end(); ++paramIt) {
			ss << (*paramIt) << ",";
		}

	return ss.str();
}

bool changeFluent(bwi_kr::ChangeFluent::Request &req,
						bwi_kr::ChangeFluent::Response &res) {
	
	//TODO check that the fluent exists and has the correct number of parameters
	
	stringstream ss;
	
	ss << req.fluent.name << "(" << concatenateParameters(req.fluent.parameters) << "1)." << endl;
	createCurrentState(ss.str());
	
	return true;
}



void createInitialstate(const std::string& initialFile) {
	
	system(("rm -f " + packagePath + domainName + "/current.asp").c_str());
	system(("cp " + initialFile + " " + packagePath + domainName + "/initial.asp").c_str());
	
	createCurrentState("");
	
	system(("rm -f " + packagePath + domainName + "/initial.asp").c_str());
	
}


void createCurrentState(const std::string& observations) {
	
	const string queryPath = "/tmp/bwi_kr_currentstate_query.txt";
	
	stringstream copyCommand;
	copyCommand << "cp " << packagePath + domainName + "/stateQuery.txt " << queryPath;
	
	system(copyCommand.str().c_str());
	
	ofstream queryFile(queryPath.c_str(),ios::app);
	
	queryFile << observations << endl;
	queryFile.close();
	
	ComputeAnswerSet answer;
	answer.request.queryFile = queryPath;
	
	computeAnswerSet(answer.request,answer.response);
	
	ofstream currentFile((packagePath + domainName + "/current.asp").c_str());
	
	vector<Predicate> &res = answer.response.answerSet.predicates;
	
	vector<Predicate>::const_iterator resIt = res.begin();
	
	for(; resIt != res.end(); ++resIt) {
	
		if(resIt->timeStep == 1) {
			currentFile << resIt->name << "(";
			currentFile << concatenateParameters(resIt->parameters);
			currentFile << "0)." << endl;
		}
	}
}
