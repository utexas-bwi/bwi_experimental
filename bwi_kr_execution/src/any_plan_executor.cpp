
#include "msgs_utils.h"
#include "RemoteReasoner.h"

#include "actasp/action_utils.h"
#include "actasp/executors/ReplanningActionExecutor.h"
#include "actasp/planners/AnyPlan.h"
#include "actasp/CostLearner.h"
#include "actasp/ExecutionObserver.h"
#include "actasp/PlanningObserver.h"
#include "actasp/AnswerSet.h"


#include "bwi_kr_execution/ExecutePlanAction.h"

#include "actions/ActionFactory.h"
#include "actions/LogicalNavigation.h"

#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>

#include <string>

#include <sys/stat.h>
#include <unistd.h>

const int MAX_N = 20;
const std::string queryDirectory("/tmp/bwi_action_execution/");


using namespace std;
using namespace bwi_krexec;
using namespace actasp;

typedef actionlib::SimpleActionServer<bwi_kr_execution::ExecutePlanAction> Server;


ActionExecutor *executor;

struct PrintFluent {
  
  PrintFluent(ostream& stream) : stream(stream) {}
  
  string operator()(const AspFluent& fluent) {
    stream << fluent.toString() << " ";
  }
  
  ostream &stream;
  
};

class ExponentialWeightedCostLearner : public CostLearner {

  public:
    ExponentialWeightedCostLearner(const std::map<std::string, int>& action_names_to_num_params_map, 
                                   float alpha = 0.5f) : 
        CostLearner(action_names_to_num_params_map), alpha(alpha) {}

    bool addSample(const AspFluent& action, 
                   const std::set<AspFluent>& currentState, 
                   bool failed,
                   float cost) {
      AspFluent actionNoTimestamp(action.getName(), action.getParameters());
      float origCost = (costs.find(actionNoTimestamp) != costs.end()) ? costs[actionNoTimestamp] : 1.f;
      float newCost = (1 - alpha) * origCost + alpha * cost;
      costs[actionNoTimestamp] = newCost;
    }

    static std::map<std::string, int> getActionNamesToNumParamsMap() {
      // TODO use the string from the action messages here.
      std::map<std::string, int> return_map;
      return_map["approach"] = 3;
      return_map["askperson"] = 1;
      return_map["gothrough"] = 1;
      return_map["opendoor"] = 2;
      return_map["remind"] = 0;
      return_map["searchroom"] = 1;
      return return_map;
    }

    // TODO define this function.
    static std::vector<std::string> getCostParametersForAction(const AspFluent& action,
                                                               const std::set<AspFluent>& currentState);

  private:
    float alpha;

};

class Observer : public ExecutionObserver, public PlanningObserver {
  
  public:

    Observer(const std::string domainDirectory) : 
        domainDirectory(domainDirectory), 
        learner(ExponentialWeightedCostLearner::getActionNamesToNumParamsMap()) {

      // Check if the symbolic link for the yaml file exists and can be resolved.
      std::string yaml_costs_file = domainDirectory + "costs.yaml";
      if (file_exists(yaml_costs_file)) {
        learner.initializeCostsFromValuesFile(yaml_costs_file);
        // Resolve link.
        char resolved_link[512];
        int count = readlink(yaml_costs_file.c_str(), resolved_link, sizeof(resolved_link));
        if (count >= 0) {
          resolved_link[count] = '\0';
        }
        // Extract counter from file name.
        for (int i = count - 1; i >= 0; --i) {
          if (resolved_link[i] == '.') {
            char temp[i];
            strncpy(temp, resolved_link + i + 1, count - i - 1);
            counter = atoi(temp);
            break;
          }
        }
      } else {
        counter = 0;
      }
      updateCostsFile();
    }
    
    void actionStarted(const AspFluent& action, const std::set<AspFluent>& currentState) throw() {
      ROS_INFO_STREAM("Starting execution: " << action.toString());
      actionStartTime = ros::Time::now().toSec();
      originalState = currentState;
    }
    
    void actionTerminated(const AspFluent& action, bool failed) throw() {
      ROS_INFO_STREAM("Terminating execution: " << action.toString());
      learner.addSample(action, originalState, failed, ros::Time::now().toSec() - actionStartTime);
    }
    
    void planExecutionFailed() throw() {
      ++counter;
      updateCostsFile();
    }
    
    void planExecutionSucceeded() throw() {
      ++counter;
      updateCostsFile();
    }

    void planChanged(const AnswerSet& newPlan) throw() {
     stringstream planStream;
     
     ROS_INFO_STREAM("plan size: " << newPlan.getFluents().size());
     
     copy(newPlan.getFluents().begin(),newPlan.getFluents().end(),ostream_iterator<string>(planStream," "));
     
     ROS_INFO_STREAM(planStream.str());
    }
  
  private:

    void updateCostsFile() {
      if (counter == 0) {
        mkdir((domainDirectory + "yaml_costs/").c_str(), 0755);
      }
      std::stringstream current_yaml_file_ss;
      current_yaml_file_ss << domainDirectory << "yaml_costs/costs.yaml." << counter;
      std::string current_yaml_file = current_yaml_file_ss.str();
      learner.writeValuesFile(current_yaml_file);
      std::string yaml_symlink_file = domainDirectory + "costs.yaml";
      unlink(yaml_symlink_file.c_str());
      int retno = symlink(current_yaml_file.c_str(), yaml_symlink_file.c_str());
      learner.writeLuaFile(domainDirectory + "costlua.asp");
    }

    bool file_exists(const std::string& name) {
      struct stat buffer;   
      return (stat (name.c_str(), &buffer) == 0); 
    }

    long start_time;
    std::string domainDirectory;
    ExponentialWeightedCostLearner learner;
    int counter;
    
    double actionStartTime;
    std::set<AspFluent> origState;

};

void executePlan(const bwi_kr_execution::ExecutePlanGoalConstPtr& plan, Server* as) {

  vector<AspRule> goalRules;

  transform(plan->aspGoal.begin(),plan->aspGoal.end(),back_inserter(goalRules),TranslateRule());

  executor->setGoal(goalRules);

  ros::Rate loop(10);

  while (!executor->goalReached() && !executor->failed() && ros::ok()) {

    if (!as->isPreemptRequested()) {
      executor->executeActionStep();
    }
    else {
      if(as->isNewGoalAvailable()) {
        goalRules.clear();
        const bwi_kr_execution::ExecutePlanGoalConstPtr& newGoal = as->acceptNewGoal();
        transform(newGoal->aspGoal.begin(),newGoal->aspGoal.end(),back_inserter(goalRules),TranslateRule());
        executor->setGoal(goalRules);
      }
    }
         loop.sleep();
  }


  if (executor->goalReached()) {
    ROS_INFO("Execution succeded");
    as->setSucceeded();
  }
  else {
    ROS_INFO("Execution failed");
    as->setAborted();
  }
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "action_executor");
  ros::NodeHandle n;

//   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//   }
  
  ros::NodeHandle privateNode("~");
  string domainDirectory;
  n.param<std::string>("/bwi_kr_execution/domain_directory", domainDirectory, ros::package::getPath("bwi_kr_execution")+"/domain/");
  
  if(domainDirectory.at(domainDirectory.size()-1) != '/')
    domainDirectory += '/';

//  create initial state
  LogicalNavigation setInitialState("noop");
  setInitialState.run();


  bool simulating;
  privateNode.param<bool>("simulation",simulating,false);
  ActionFactory::setSimulation(simulating); 
  
  boost::filesystem::create_directories(queryDirectory);

  AspKR *reasoner = new RemoteReasoner(MAX_N,queryDirectory,domainDirectory,actionMapToSet(ActionFactory::actions()));
  Planner *planner = new AnyPlan(reasoner,1.);
  
  //need a pointer to the specific type for the observer
  ReplanningActionExecutor *replanner = new ReplanningActionExecutor(reasoner,planner,ActionFactory::actions());
  executor = replanner;
  
  // Get the set of action names (required by the cost learning observer).
  Observer observer(domainDirectory);
  executor->addExecutionObserver(&observer);
  replanner->addPlanningObserver(&observer);

  Server server(privateNode, "execute_plan", boost::bind(&executePlan, _1, &server), false);
  server.start();

  ros::spin();

  
  return 0;
}
