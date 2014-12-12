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
               const std::string& incrementalVar,
               const std::string& queryDir,
               const std::string& domainDir,
               const ActionSet& actions,
               unsigned int max_time
              ) throw() :
  max_n(max_n),
  incrementalVar(incrementalVar),
  max_time(max_time),
  queryDir(queryDir),
  domainDir(domainDir),
  allActions(actions),
  actionFilter() {

  if (max_time > 0 && !system("timeout 2>/dev/null")) //make sure timeout is available
    max_time = 0;

  //make sure directory ends with '/'

  if (this->queryDir.find_last_of("/") != (this->queryDir.length() -1))
    this->queryDir += "/";

  if ((this->domainDir.find_last_of("/")) != (this->domainDir.length() -1))
    this->domainDir += "/";

  //TODO test the existance of the directories

  stringstream filterStream;
  filterStream << "#hide." << endl;

  std::set<AspFluent>::const_iterator actIt = allActions.begin();
  for (; actIt != allActions.end(); ++actIt) {
    filterStream << "#show " << actIt->getName() << "/" << actIt->arity() << "." << endl;
  }

  actionFilter = filterStream.str();
}

struct RuleToString {
  RuleToString(unsigned int timeStepNum) {
    stringstream ss;
    ss << timeStepNum;
    timeStep = ss.str();
  }

  RuleToString(const string& timeStepVar) : timeStep(timeStepVar) {}

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

  string timeStep;
};


static string aspString(const std::vector<actasp::AspRule>& query, const string& timeStepVar) {

  stringstream aspStream;
  transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToString(timeStepVar));
  return aspStream.str();
}

static string aspString(const std::vector<actasp::AspRule>& query, unsigned int timeStep) {

  stringstream vs;
  vs << timeStep;

  return aspString(query,vs.str());
}


struct CreateFluent {
  AspFluent operator()(const std::string & fluentDescription) const {
    return AspFluent(fluentDescription);
  }
};

static std::set<actasp::AspFluent> parseAnswerSet(const std::string& answerSetContent) {

  stringstream predicateLine(answerSetContent);

  set<AspFluent> predicates;

  //split the line based on spaces
  transform(istream_iterator<string>(predicateLine),
            istream_iterator<string>(),
            inserter(predicates,predicates.begin()),
            CreateFluent());

  return predicates;
}

static std::vector<actasp::AnswerSet> readAnswerSets(const std::string& filePath) {

  ifstream file(filePath.c_str());

  stringstream answerSetContent;

  copy(istreambuf_iterator<char>(file),
       istreambuf_iterator<char>(),
       ostreambuf_iterator<char>(answerSetContent));

  size_t pos = answerSetContent.str().find_last_of("SATISFIABLE");

  bool satisfiable = answerSetContent.str().substr(pos-2,13) != "UNSATISFIABLE";

  vector<AnswerSet> allSets;

  if (satisfiable) {

    string firstLine;
    string eachAnswerset;

    while (answerSetContent) {

      getline(answerSetContent,firstLine);
      if (firstLine.find("Answer") != string::npos) {
        getline(answerSetContent,eachAnswerset);
        try {
          set<AspFluent> fluents = parseAnswerSet(eachAnswerset);
          allSets.push_back(AnswerSet(true, fluents));
        } catch (std::invalid_argument& arg) {
          //swollow it and skip this answer set.
        }
      }
    }
  }

  return allSets;
}

std::vector<actasp::AnswerSet> Clingo::krQuery(const std::string& query,
    unsigned int initialTimeStep,
    unsigned int finalTimeStep,
    const std::string& fileName,
    unsigned int answerSetsNumber = 1) const throw() {


  //this depends on our way of representing stuff.
  //iclingo starts from 1, while we needed the initial state and first action to be at time step 0
  initialTimeStep++;
  finalTimeStep++;

  string queryPath = queryDir + fileName;

  ofstream queryFile(queryPath.c_str());
  queryFile << query << endl;
  queryFile.close();

  stringstream commandLine;

  const string outputFilePath = queryDir + "query_output.txt";

  if (max_time > 0) {
    commandLine << "timeout " << max_time << " ";
  }

  stringstream iterations;
  iterations << "--imin=" << initialTimeStep << " --imax=" << finalTimeStep;


  commandLine << "iclingo " << iterations.str() << " " << queryPath << " " << domainDir << "*.asp " << " > " << outputFilePath << " " << answerSetsNumber;


  if (!system(commandLine.str().c_str())) {
    //maybe do something here, or just kill the warning about the return value not being used.
  }

  return readAnswerSets(outputFilePath);

}

string Clingo::generatePlanQuery(const std::vector<actasp::AspRule>& goalRules,
                                 bool filterActions = true) const throw() {
  stringstream goal;
  goal << "#volatile " << incrementalVar << "." << endl;
  //I don't like this -1 too much, but it makes up for the incremental variable starting at 1
  goal << aspString(goalRules,incrementalVar+"-1") << endl;

  if (filterActions)
    goal << actionFilter;


  return goal.str();
}

//TODO continue here

AnswerSet Clingo::computePlan(const std::vector<actasp::AspRule>& goal) const throw() {

  vector<AnswerSet> answerSets;

  string query = generatePlanQuery(goal);

  answerSets = krQuery(query,0,max_n,"planQuery.asp");


  if (answerSets.empty())
    return AnswerSet(false);

  return  answerSets[0];
}

struct PolicyMerger {

  PolicyMerger(MultiPolicy & policy) :
    policy(policy) {}

  void operator()(const AnswerSet& set) {
    policy.merge(set);
  }

  MultiPolicy &policy;
};

//substituted with the check on sub-sequences

// static set<AspFluent> reducePlan(const list<AspFluent> &plan, int begin, int length) {
//
//   set<AspFluent> newPlan;
//
//   list<AspFluent>::const_iterator actionIt = plan.begin();
//
//   for (int timeStep = 0; actionIt != plan.end(); ++timeStep, ++actionIt) {
//     if (timeStep < begin) {
//       newPlan.insert(*actionIt);
//     } else if (timeStep >= begin + length) {
//       AspFluent fluentCopy(*actionIt);
//       fluentCopy.setTimeStep(timeStep - length);
//       newPlan.insert(fluentCopy);
//     }
//   }
//
//   return newPlan;
// }
//
// struct IsNotLocallyOptimal {
//
//   IsNotLocallyOptimal(const Clingo *reasoner,
//                       const std::vector<actasp::AspRule>& goal,
//                       const ActionSet &allActions ) : reasoner(reasoner), goal(goal), allActions(allActions) {}
//
//   bool operator()(const AnswerSet& plan) const {
//
//     list<AspFluent> actionsOnly;
//
//     remove_copy_if(plan.getFluents().begin(), plan.getFluents().end(),
//                      back_inserter(actionsOnly),not1(IsAnAction(allActions)));
//
//     for (int length = 1; length < actionsOnly.size(); ++length) {
//
//       for (int begin = 0; begin <= actionsOnly.size() - length; ++begin) {
//
//         //remove length actions from begin
//         set<AspFluent> reduced = reducePlan(actionsOnly,begin,length);
//
//         if (reasoner->isPlanValid(AnswerSet(true,reduced),goal))
//           return true;
//
//       }
//     }
//
//     return false;
//   }
//
//   const Clingo *reasoner;
//   const vector<actasp::AspRule>& goal;
//   const ActionSet &allActions;
// };


//checks wheter the shorter plan occurs in the longer one. For instance: ABCDEF and ABCDXYEF removing XY

struct IsSubSequence {
  
  IsSubSequence(const list< AspFluent> &myPlan) : longerPlan(myPlan) {};
  
  bool operator()(const list<AspFluent> &shorterPlan) const {
    
    pair< list< AspFluent>::const_iterator, list< AspFluent>::const_iterator>
    start = mismatch(shorterPlan.begin(),shorterPlan.end(),longerPlan.begin(), ActionEquality());
    
    size_t remaining = distance(start.first, shorterPlan.end());
    list< AspFluent>::const_iterator secondStart = longerPlan.end();
    advance(secondStart, -remaining);
    
    return equal(start.first, shorterPlan.end(), secondStart, ActionEquality());
    
  }
  
  const list< AspFluent> &longerPlan;
  
};


struct IsNotLocallyOptimalSubPlanCheck {

  IsNotLocallyOptimalSubPlanCheck(const list< list< AspFluent> > &allPlans, const ActionSet &allActions) :
    allPlans(allPlans), allActions(allActions) {}

  bool operator()(const AnswerSet& plan) const {
    list<AspFluent> actionsOnly;

    remove_copy_if(plan.getFluents().begin(), plan.getFluents().end(),
                   back_inserter(actionsOnly),not1(IsAnAction(allActions)));

    list< list< AspFluent> >::const_iterator sub = find_if(allPlans.begin(),allPlans.end(),IsSubSequence(actionsOnly));

    return sub != allPlans.end();
  }

  const list< list< AspFluent> > &allPlans;
  const ActionSet &allActions;
};

struct CleanPlan {

  CleanPlan(const ActionSet &allActions) : allActions(allActions) {}

  list<AspFluent> operator()(const AnswerSet &planWithStates) const {
    list<AspFluent> actionsOnly;

    remove_copy_if(planWithStates.getFluents().begin(), planWithStates.getFluents().end(),
                   back_inserter(actionsOnly),not1(IsAnAction(allActions)));

    return actionsOnly;

  }

  const ActionSet &allActions;

};

struct PlanLongerThan {

  PlanLongerThan(unsigned int length) : length(length) {}

  bool operator()(const AnswerSet& plan) const {
    return plan.maxTimeStep() > length;
  }

  unsigned int length;
};

MultiPolicy Clingo::computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error) {

  if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
  }

  string query = generatePlanQuery(goal,false);

  vector<AnswerSet> answerSets = krQuery(query,0,max_n,"planQuery.asp",0);

  MultiPolicy policy(allActions);

  if (answerSets.empty())
    return policy;

  unsigned int shortestLength = answerSets[0].maxTimeStep();

  for_each(answerSets.begin(),answerSets.end(),PolicyMerger(policy));

  list< list <AspFluent> > allPlans;

  //remove the states from the plans
  transform(answerSets.begin(),answerSets.end(),back_inserter(allPlans), CleanPlan(allActions));

  int maxLength = floor(suboptimality * shortestLength);

  if (maxLength == shortestLength)
    return policy;

  query = generatePlanQuery(goal,false);
  answerSets = krQuery(query,maxLength,maxLength,"planQuery.asp",0);

  vector<AnswerSet>::iterator currentFirst = answerSets.begin();

  while (currentFirst != answerSets.end()) {

    //process the plans in groups of increasing length

    vector<AnswerSet>::iterator currentLast = find_if(currentFirst,answerSets.end(),PlanLongerThan(currentFirst->maxTimeStep()));

    vector<AnswerSet>::iterator newEnd = remove_if(currentFirst,currentLast,IsNotLocallyOptimalSubPlanCheck(allPlans,allActions));

    transform(currentFirst,newEnd,back_inserter(allPlans), CleanPlan(allActions));

//     vector<AnswerSet>::iterator secondFilter = remove_if(answerSets.begin(),answerSets.end(),IsNotLocallyOptimal(this,goal,allActions));
//     answerSets.erase(secondFilter,answerSets.end());

    for_each(answerSets.begin(),newEnd,PolicyMerger(policy));

    currentFirst = currentLast;

  }

  return policy;

}

struct AnswerSetToList {
  list <AspFluent> operator()(const AnswerSet& aset) const {

    return list <AspFluent>(aset.getFluents().begin(), aset.getFluents().end());

  }
};

struct ListToAnswerSet {
  AnswerSet operator()(const list<AspFluent>& plan) {
    return AnswerSet(true, set<AspFluent> (plan.begin(), plan.end()));
  }

};

std::vector< AnswerSet > Clingo::computeAllPlans(const std::vector<actasp::AspRule>& goal,
    double suboptimality) const throw () {

  if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
  }

  string query = generatePlanQuery(goal,true);
  vector<AnswerSet> answerSets = krQuery(query,0,max_n,"planQuery.asp",0);

  if (answerSets.empty())
    return vector<AnswerSet>();

  //when actions are filtered and there are not state fluents,
  //the last time step is of the last action, and actions start at
  //zero, so we need +1
  unsigned int shortestLength = answerSets[0].maxTimeStep()+1;

  list< list<AspFluent> > allPlansList;
  transform(answerSets.begin(), answerSets.end(),back_inserter(allPlansList),AnswerSetToList());

  vector< AnswerSet> finalVector(answerSets);

  int maxLength = floor(suboptimality * shortestLength);

  if (maxLength == shortestLength)
    return finalVector;

  query = generatePlanQuery(goal,true);
  answerSets = krQuery(query,maxLength,maxLength,"planQuery.asp",0);

  vector<AnswerSet>::iterator currentFirst = answerSets.begin();
  //the first group are the shortest plans and have already been included, skip them.
  currentFirst = find_if(currentFirst,answerSets.end(),PlanLongerThan(currentFirst->maxTimeStep()+1));

  while (currentFirst != answerSets.end()) {

    //process the plans in groups of increasing length

    vector<AnswerSet>::iterator currentLast = find_if(currentFirst,answerSets.end(),PlanLongerThan(currentFirst->maxTimeStep()+1));

    vector<AnswerSet>::iterator newEnd = remove_if(currentFirst,currentLast,IsNotLocallyOptimalSubPlanCheck(allPlansList,allActions));

    //     vector<AnswerSet>::iterator secondFilter = remove_if(answerSets.begin(),answerSets.end(),IsNotLocallyOptimal(this,goal,allActions));
    //     answerSets.erase(secondFilter,answerSets.end());


    transform(currentFirst, newEnd,back_inserter(allPlansList),AnswerSetToList());

    finalVector.insert(finalVector.end(),currentFirst, newEnd);

    currentFirst = currentLast;

  }

  return finalVector;
}


bool Clingo::isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw() {


  string planQuery = generatePlanQuery(goal);

  stringstream monitorQuery(planQuery, ios_base::app | ios_base::out);

  const set<AspFluent> &allActions = plan.getFluents();
  set<AspFluent>::const_iterator actionIt = allActions.begin();

  for (int i=0; actionIt != allActions.end(); ++actionIt, ++i)
    monitorQuery << actionIt->toString(i) << "." << endl;

  return !(krQuery(monitorQuery.str(),plan.getFluents().size(),plan.getFluents().size(),"monitorQuery.asp").empty());
}

AnswerSet Clingo::currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() {

  vector<AnswerSet> sets = krQuery(aspString(query,0),0,0,"stateQuery.asp");

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

  vector<AnswerSet> currentState = krQuery(queryStream.str(),1,1,"observationQuery.asp");

  if (currentState.empty())
    return false; //the observations are incompatible with the current state and are discarded

  vector<AspRule> newStateRules;
  //iclingo generates all answer sets up to the last iteration, so we need the last one here
  set<AspFluent> newStateFluents = currentState[currentState.size()-1].getFluentsAtTime(1);
  newStateRules.reserve(newStateFluents.size());
  transform(newStateFluents.begin(),newStateFluents.end(),back_inserter(newStateRules),fluent2Rule);

  //copy the current state in a file
  ofstream currentFile((domainDir + CURRENT_STATE_FILE).c_str());

  currentFile << aspString(newStateRules,0);
  currentFile.close();

  return true;
}

void Clingo::reset() throw() {
  ofstream current((domainDir + CURRENT_STATE_FILE).c_str());
  current << "";
  current.close();
}

}