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

#define CURRENT_STATE_FILE "current.asp"

using namespace std;

namespace actasp {



//own the actions in the map
Clingo::Clingo(unsigned int max_n,
               const std::string& queryDir,
               const std::string& domainDir,
               const std::vector<AspFluent>& actions) throw() :
  max_n(max_n),
  queryDir(queryDir),
  domainDir(domainDir),
  allActions(actions) {


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

  if (!system(commandLine.str().c_str())) {
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

    std::vector<AspFluent>::const_iterator actIt = allActions.begin();
    for (; actIt != allActions.end(); ++actIt) {
      goal << "#show " << actIt->getName() << "/" << actIt->arity() << "." << endl;
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

  if (answerSets.empty())
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
  throw logic_error("not implemented");
  //TODO reimplement this after having changed MultiPolicy to not require the action map
//   if(suboptimality < 1) {
//     stringstream num;
//     num << suboptimality;
//     throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
//   }
//
//  vector<AnswerSet> answerSets;
//
//  unsigned int shortestLength = 0;
//
//  for (; shortestLength<this->max_n && answerSets.empty() ; ++shortestLength) {
//
//    string query = generatePlanQuery(goal,shortestLength,false);
//
//    answerSets = krQuery(query,shortestLength,"planQuery.asp",0);
//
//  }
//
//  MultiPolicy policy;
//
//  for_each(answerSets.begin(),answerSets.end(),PolicyMerger(policy,actionMap));
//
//  int maxLength = floor(suboptimality * shortestLength);
//
//  for (int i= shortestLength+1; i <= maxLength && i < max_n; ++i) {
//
//    string query = generatePlanQuery(goal,i,false);
//    answerSets = krQuery(query,i,"planQuery.asp",0);
//
//    for_each(answerSets.begin(),answerSets.end(),PolicyMerger(policy,actionMap));
//
//  }
//
//  return policy;

}

static vector<AspFluent> reducePlan(const AnswerSet& plan, int begin, int length) {

  vector<AspFluent> newPlan;
  newPlan.reserve(plan.getFluents().size() - length);

  for (int timeStep = 0; timeStep < plan.getFluents().size(); ++timeStep) {
    if (timeStep < begin) {
      newPlan.push_back(plan.getFluents()[timeStep]);
    } else if (timeStep >= begin + length) {
      AspFluent fluentCopy(plan.getFluents()[timeStep]);
      fluentCopy.setTimeStep(timeStep - length);
      newPlan.push_back(fluentCopy);

    }
  }

  return newPlan;
}

struct IsNotLocallyOptimal {

  IsNotLocallyOptimal(Clingo *reasoner, const std::vector<actasp::AspRule>& goal) : reasoner(reasoner), goal(goal) {}

  bool operator()(const AnswerSet& plan) {
    for (int length = 1; length < plan.getFluents().size(); ++length) {

      for (int begin = 0; begin <= plan.getFluents().size() - length; ++begin) {

        //remove length actions from begin
        vector<AspFluent> reduced = reducePlan(plan,begin,length);

        if (reasoner->isPlanValid(AnswerSet(true,reduced),goal))
          return true;

      }
    }

    return false;
  }

  Clingo *reasoner;
  const std::vector<actasp::AspRule>& goal;
};

std::vector< AnswerSet > Clingo::computeAllPlans(const std::vector<actasp::AspRule>& goal,
    double suboptimality) throw () {

  if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
  }
  vector<AnswerSet> answerSets;

  unsigned int shortestLength = 0;

  for (; shortestLength<this->max_n && answerSets.empty() ; ++shortestLength) {

    string query = generatePlanQuery(goal,shortestLength,true);

    answerSets = krQuery(query,shortestLength,"planQuery.asp",0);

  }

  //it's incremented before leaving the loop
  --shortestLength;

  vector<AnswerSet>::iterator newEnd = remove_if(answerSets.begin(),answerSets.end(),IsNotLocallyOptimal(this,goal));
  answerSets.erase(newEnd,answerSets.end());

  vector< AnswerSet > allPlans(answerSets);

  int maxLength = floor(suboptimality * shortestLength);


  for (int i= shortestLength+1; i <= maxLength && i < max_n; ++i) {

    string query = generatePlanQuery(goal,i,true);
    answerSets = krQuery(query,i,"planQuery.asp",0);

    vector<AnswerSet>::iterator newEnd = remove_if(answerSets.begin(),answerSets.end(),IsNotLocallyOptimal(this,goal));
    answerSets.erase(newEnd,answerSets.end());

    allPlans.insert(allPlans.end(), answerSets.begin(),answerSets.end());

  }

  return allPlans;
}


bool Clingo::isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw() {

  string planQuery = generatePlanQuery(goal,plan.getFluents().size());

  stringstream monitorQuery(planQuery, ios_base::app | ios_base::out);

  const vector<AspFluent> &allActions = plan.getFluents();

  for (int i=0, size = allActions.size(); i <size; ++i)
    monitorQuery << allActions[i].toString(i) << "." << endl;

  return !(krQuery(monitorQuery.str(),plan.getFluents().size(),"monitorQuery.asp").empty());
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

}