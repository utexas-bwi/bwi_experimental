#include <actasp/reasoners/Clingo.h>

#include <actasp/AnswerSet.h>
#include <actasp/action_utils.h>
#include <actasp/Action.h>


#include <sstream>
#include <ostream>
#include <iterator>
#include <algorithm>
#include <fstream>
#include <cmath> //for floor
#include <iostream>

#define CURRENT_STATE_FILE "current.asp"

using namespace std;

namespace actasp {
	


//own the actions in the map
Clingo::Clingo(unsigned int max_n,
               const std::string& queryDir,
               const std::string& domainDir,
               const std::map<std::string, actasp::Action*>& actionMap) throw() :
	max_n(max_n),
	queryDir(queryDir),
	domainDir(domainDir),
	actionMap() {
		
	transform(actionMap.begin(),actionMap.end(),inserter(this->actionMap, this->actionMap.begin()),ActionMapDeepCopy());

	//make sure directory end with '/'

	if (this->queryDir.find_last_of("/") != (this->queryDir.length() -1))
		this->queryDir += "/";

	if ((this->domainDir.find_last_of("/")) != (this->domainDir.length() -1))
		this->domainDir += "/";

	//TODO test the existance of the directories

	//clear up the current state file
	ofstream current((domainDir + CURRENT_STATE_FILE).c_str());
	current << "";
	current.close();
}

struct RuleToString {
	RuleToString(unsigned int timeStep) : timeStep(timeStep) {}
	std::string operator()(const AspRule& rule) {

		stringstream ruleStream;

		//iterate over head
		for (int i =0, size = rule.head.size(); i <size; ++i) {
			ruleStream << rule.head[i].toString(timeStep);
			if (i < (size-1))
				ruleStream << ", ";
		}

		if (!(rule.body.empty()))
			ruleStream << ":- ";

		//iterate over body
		for (int i =0, size = rule.body.size(); i <size; ++i) {
			ruleStream << rule.body[i].toString(timeStep);
			if (i < (size-1))
				ruleStream << ", ";
		}

		if (!(rule.head.empty() && rule.body.empty()))
			ruleStream << "." << std::endl;

		return ruleStream.str();
	}

	unsigned int timeStep;
};

static string aspString(const std::vector<actasp::AspRule>& query, unsigned int timeStep) {

	stringstream aspStream;
	transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToString(timeStep));
	return aspStream.str();
}


struct CreateFluent {
	AspFluent operator()(const std::string & fluentDescription) {
		return AspFluent(fluentDescription);
	}
};

static std::vector<actasp::AspFluent> parseAnswerSet(const std::string& answerSetContent) {
	stringstream content(answerSetContent);

	string line;
	getline(content,line); //the first line contains the sequence number of the answer set.
	getline(content,line);

	stringstream predicateLine(line);

	vector<AspFluent> predicates;

	//split the line based on spaces
	transform(istream_iterator<string>(predicateLine),
	          istream_iterator<string>(),
	          back_inserter<vector<AspFluent> >(predicates),
	          CreateFluent());

	return predicates;
}

static std::vector<actasp::AnswerSet> readAnswerSets(const std::string& filePath) {

	ifstream file(filePath.c_str());

	string line;
	string answerSetContent;
	while (file) {
		getline(file, line);
		answerSetContent += line;
		answerSetContent += "\n";
	}

	bool satisfiable = answerSetContent.find("UNSATISFIABLE") == string::npos;

	vector<AnswerSet> allSets;

	if (satisfiable) {

		stringstream content(answerSetContent);

		string firstLine;
		string eachAnswerset;

		while (content) {

			getline(content,firstLine);
			if (firstLine.find("Answer") != string::npos) {
				getline(content,eachAnswerset);
				vector<AspFluent> fluents = parseAnswerSet(eachAnswerset);
				allSets.push_back(AnswerSet(true, fluents));
			}
		}
	}

	return allSets;
}

std::vector<actasp::AnswerSet> Clingo::krQuery(const std::string& query,
        unsigned int timeStep,
        const std::string& fileName,
        unsigned int answerSetsNumber = 1) const throw() {

	string queryPath = queryDir + fileName;

	ofstream queryFile(queryPath.c_str());
	queryFile << "#const n=" << timeStep<< "." << endl;
	queryFile << query << endl;
	queryFile.close();

	stringstream commandLine;

	const string outputFilePath = queryDir + "query_output.txt";

	commandLine << "clingo " << queryPath << " " << domainDir << "*.asp " << " > " << outputFilePath << " " << answerSetsNumber;

	if(!system(commandLine.str().c_str())) {
			//TODO do something if clingo fails
	}

	return readAnswerSets(outputFilePath);
}

string Clingo::generatePlanQuery(const std::vector<actasp::AspRule>& goalRules,
                                 unsigned int timeStep,
                                 bool filterActions = true) const throw() {
	stringstream goal;
	goal << aspString(goalRules,timeStep) << endl;

	if (filterActions) {
		goal << "#hide." << endl;

		std::map<std::string, Action * >::const_iterator actIt = actionMap.begin();
		for (; actIt != actionMap.end(); ++actIt) {
			//the last parameter is always the the step number
			goal << "#show " << actIt->second->getName() << "/" << actIt->second->paramNumber() + 1 << "." << endl;
		}
	}

	return goal.str();
}

AnswerSet Clingo::computePlan(const std::vector<actasp::AspRule>& goal) throw() {

	vector<AnswerSet> answerSets;

	for (int i=0; i<this->max_n && answerSets.empty() ; ++i) {
		string query = generatePlanQuery(goal,i);

		answerSets = krQuery(query,i,"planQuery.asp");

	}

	if(answerSets.empty())
		return AnswerSet(false);
	
	return  answerSets[0];
}

struct PolicyMerger {

	PolicyMerger(MultiPolicy & policy,  std::map<std::string, Action * > &actions) :
		policy(policy), actions(actions) {}

	void operator()(const AnswerSet& set) {
		policy.merge(set,actions);
	}

	MultiPolicy &policy;
	std::map<std::string, Action * > &actions;
};

MultiPolicy Clingo::computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) throw (std::logic_error) {
	//TODO check the range of suboptimality

	vector<AnswerSet> answerSets;

	unsigned int shortestLength = 0;

	for (; shortestLength<this->max_n && answerSets.empty() ; ++shortestLength) {

		string query = generatePlanQuery(goal,shortestLength,false);

		answerSets = krQuery(query,shortestLength,"planQuery.asp",0);

	}

	MultiPolicy policy;

	for_each(answerSets.begin(),answerSets.end(),PolicyMerger(policy,actionMap));

	int maxLength = floor(suboptimality * shortestLength);

	for (int i= shortestLength+1; i <= maxLength && i < max_n; ++i) {

		string query = generatePlanQuery(goal,i,false);
		answerSets = krQuery(query,i,"planQuery.asp",0);

		for_each(answerSets.begin(),answerSets.end(),PolicyMerger(policy,actionMap));

	}

	return policy;

}

std::vector< AnswerSet > Clingo::computeAllPlans(	const std::vector<actasp::AspRule>& goal, 
															double suboptimality) throw () {
	//TODO check the range of suboptimality
	vector<AnswerSet> answerSets;

	unsigned int shortestLength = 0;

	for (; shortestLength<this->max_n && answerSets.empty() ; ++shortestLength) {

		string query = generatePlanQuery(goal,shortestLength,true);

		answerSets = krQuery(query,shortestLength,"planQuery.asp",0);

	}
	
	vector< AnswerSet > allPlans(answerSets);

	int maxLength = floor(suboptimality * shortestLength);


	for (int i= shortestLength+1; i <= maxLength && i < max_n; ++i) {

		string query = generatePlanQuery(goal,i,true);
		answerSets = krQuery(query,i,"planQuery.asp",0);

		allPlans.insert(allPlans.end(), answerSets.begin(),answerSets.end());

	}

	return allPlans;
}

bool Clingo::isPlanValid(std::list<actasp::Action*> plan, const std::vector<actasp::AspRule>& goal) const throw() {

	string planQuery = generatePlanQuery(goal,plan.size());

	stringstream monitorQuery(planQuery, ios_base::app | ios_base::out);

	//add the current actions
	list<Action*>::const_iterator actIt = plan.begin();

	for (int timeStep =0; actIt != plan.end(); ++actIt, ++timeStep) {
		monitorQuery << (*actIt)->toASP(timeStep) << "." << endl;

	}

	return !(krQuery(monitorQuery.str(),plan.size(),"monitorQuery.asp").empty());
}

bool Clingo::isPlanValid(AnswerSet plan, const std::vector<actasp::AspRule>& goal)  const throw() {
	//TODO implement the other in terms of this rather than the other way around
	list<Action *> planActs = plan.instantiateActions(actionMap);
	bool valid = this->isPlanValid(planActs,goal);
	for_each(planActs.begin(),planActs.end(),ActionDeleter());
	return valid;
}

AnswerSet Clingo::currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() {

	vector<AnswerSet> sets = krQuery(aspString(query,0),0,"stateQuery.asp");
	
	return (sets.empty())? AnswerSet(false) : sets[0];
}

static AspRule fluent2Rule(const AspFluent& fluent) {
	AspRule rule;
	rule.head.push_back(fluent);
	return rule;
}

bool Clingo::updateFluents(const std::vector<actasp::AspFluent> &observations) throw() {

	//copy the observations in the heads of rules
	vector<AspRule> obsRules;
	obsRules.reserve(observations.size());
	transform(observations.begin(),observations.end(),back_inserter(obsRules),fluent2Rule);
	//add the rule for the noop action

	stringstream queryStream(aspString(obsRules,1), ios_base::app | ios_base::out);

	queryStream << "noop(0)." << endl;
	queryStream << "#hide noop/1." << endl;

	vector<AnswerSet> currentState = krQuery(queryStream.str(),1,"observationQuery.asp");

	if (currentState.empty())
		return false; //the observations are incompatible with the current state and are discarded

	vector<AspRule> newStateRules;
	vector<AspFluent> newStateFluents = currentState[0].getFluentsAtTime(1);
	newStateRules.reserve(newStateFluents.size());
	transform(newStateFluents.begin(),newStateFluents.end(),back_inserter(newStateRules),fluent2Rule);

	//copy the current state in a file
	ofstream currentFile((domainDir + CURRENT_STATE_FILE).c_str());
	currentFile << aspString(newStateRules,0);
	currentFile.close();

	return true;
}

Clingo::~Clingo() {
	for_each(actionMap.begin(),actionMap.end(),ActionMapDelete());
}
}