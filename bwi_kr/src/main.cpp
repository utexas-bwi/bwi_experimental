
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <sstream>
#include <cstdlib>
#include <fstream>

#include <bwi_kr/AnswerSet.h>
#include <bwi_kr/ComputeAnswerSet.h>
#include <bwi_kr/ChangeFluent.h>
#include <std_srvs/Empty.h>
#include <boost/thread.hpp>
#include <ros/spinner.h>

using namespace std;
using namespace bwi_kr;

const std::string domainName = "domain";
string packagePath;

static void createCurrentState(const std::string& observations);
static void createInitialstate(const std::string& initialFile);

boost::shared_mutex mutex;

bool computeAnswerSet(bwi_kr::ComputeAnswerSet::Request& req,
						bwi_kr::ComputeAnswerSet::Response &res);

bool changeFluent(bwi_kr::ChangeFluent::Request &req,
						bwi_kr::ChangeFluent::Response &res);
bool printService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

AnswerSet doComputeAnswerSet(const string& queryFilereq);

int main(int argc, char **argv) {

	ros::init(argc, argv, "bwi_kr");
	ros::NodeHandle n("~");
	
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	
	packagePath = ros::package::getPath("bwi_kr")+"/";
	
	createInitialstate(packagePath + domainName + "/initial.txt");

	ros::ServiceServer computeAS = n.advertiseService("compute_answer_set", computeAnswerSet);
	//TODO consider accepting a vector of Predicates
	ros::ServiceServer changeF = n.advertiseService("change_fluent", changeFluent);
	ros::ServiceServer empty = n.advertiseService("empty", printService);
	
	ros::MultiThreadedSpinner m;
	
	ros::spin(m);

	return 0;
}

bool printService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
	ROS_INFO_STREAM("[thread=" << boost::this_thread::get_id() << "]");
	ros::Duration (60).sleep();
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
	
	boost::shared_lock<boost::shared_mutex> lock(mutex);

	AnswerSet answer  = doComputeAnswerSet(req.queryFile);	

	res.answerSet.satisfied = answer.isSatisfied();
	res.answerSet.predicates = answer.getPredicates();

	return true;

}

AnswerSet doComputeAnswerSet(const string& queryFile) {
	
	stringstream outputFileNameStream;
	outputFileNameStream << "/tmp/bwi_kr_query_output" << boost::this_thread::get_id() << ".txt";

	const string outputFilePath(outputFileNameStream.str());

	stringstream commandLine;
	commandLine << "clingo " << queryFile << " " << packagePath << domainName << "/*.asp " << " > " << outputFilePath;
		
	system(commandLine.str().c_str());
	
	return readAnswerSet(outputFilePath);
	

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

	boost::upgrade_lock<boost::shared_mutex> lock(mutex);
	boost::upgrade_to_unique_lock<boost::shared_mutex> uniquelock(lock);
	

	stringstream ss;
    //std::string str1 = "";
    //for( int i = 0; i < req.fluent.size() ; i ++ ){
	//str1 = str1 +  req.fluent[i].name + "(" +  concatenateParameters(req.fluent[i].parameters) + "1)." + "\n";//+ std::endl;
    //}
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
	
    ROS_DEBUG_STREAM( "-----------Observations string: " << observations);
	const string queryPath = "/tmp/bwi_kr_currentstate_query.txt";
	
	stringstream copyCommand;
	copyCommand << "cp " << packagePath + domainName + "/stateQuery.txt " << queryPath;
	
	system(copyCommand.str().c_str());
	
	ofstream queryFile(queryPath.c_str(),ios::app);
	
	queryFile << observations << endl;
	queryFile.close();
	

	AnswerSet answerSet = doComputeAnswerSet(queryPath);

	
	ofstream currentFile((packagePath + domainName + "/current.asp").c_str());
	
	const vector<Predicate> &res = answerSet.getPredicates();
	
	vector<Predicate>::const_iterator resIt = res.begin();
	
	for(; resIt != res.end(); ++resIt) {
	
		if(resIt->timeStep == 1) {
			currentFile << resIt->name << "(";
			currentFile << concatenateParameters(resIt->parameters);
			currentFile << "0)." << endl;
		}
	}
}
