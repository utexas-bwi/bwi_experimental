
#include "actions/ActionFactory.h"

#include "actasp/reasoners/Clingo.h"

#include "msgs_utils.h"
#include "bwi_kr_execution/UpdateFluents.h"
#include "bwi_kr_execution/CurrentStateQuery.h"
#include "bwi_kr_execution/ComputePlan.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>

using namespace actasp;
using namespace std;
using namespace ros;
using namespace bwi_krexec;

const int MAX_N = 50;
const std::string queryDirectory("/tmp/bwi_kr_execution/");



bool updateFluents(bwi_kr_execution::UpdateFluents::Request  &req,
                   bwi_kr_execution::UpdateFluents::Response &res);

bool currentStateQuery(bwi_kr_execution::CurrentStateQuery::Request  &req,
                       bwi_kr_execution::CurrentStateQuery::Response &res);

bool computePlan(bwi_kr_execution::ComputePlan::Request  &req,
                 bwi_kr_execution::ComputePlan::Response &res);

actasp::AspKR *reasoner;

int main(int argc, char **argv) {

  ros::init(argc, argv, "bwi_kr");
  ros::NodeHandle n;

 if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
 }

  const string domainDirectory(ros::package::getPath("bwi_kr_execution")+"/domain_simulation/"); //TODO get this from a parameter
  ActionFactory::setSimulation(true);

  boost::filesystem::create_directories(queryDirectory);

  reasoner = new Clingo(MAX_N,queryDirectory,domainDirectory,ActionFactory::actions());

  ros::ServiceServer update_fluents = n.advertiseService("update_fluents", updateFluents);
  ros::ServiceServer current_state_query = n.advertiseService("current_state_query", currentStateQuery);
  ros::ServiceServer compute_plan = n.advertiseService("compute_plan", computePlan);


  ros::MultiThreadedSpinner m(2); //we don't really want to potentially block all the available cores

  ros::spin(m);

  delete reasoner;

  return 0;
}

bool updateFluents(bwi_kr_execution::UpdateFluents::Request  &req,
                   bwi_kr_execution::UpdateFluents::Response &res) {

  vector<AspFluent> fluents;
  transform(req.fluents.begin(),req.fluents.end(),back_inserter(fluents),TranslateFluent());
  
  res.consistent = reasoner->updateFluents(fluents);

  return true;
}

bool currentStateQuery(bwi_kr_execution::CurrentStateQuery::Request  &req,
                       bwi_kr_execution::CurrentStateQuery::Response &res) {

  vector<AspRule> rules;
  transform(req.query.begin(),req.query.end(),back_inserter(rules),TranslateRule());
  
  AnswerSet answer = reasoner->currentStateQuery(rules);
  
  res.answer.satisfied = answer.isSatisfied();
  transform(answer.getFluents().begin(),answer.getFluents().end(),back_inserter(res.answer.fluents),TranslateFluent());
  
  return true;
}

bool computePlan(bwi_kr_execution::ComputePlan::Request  &req,
                 bwi_kr_execution::ComputePlan::Response &res) {
  vector<AspRule> goal;
  transform(req.goal.begin(),req.goal.end(),back_inserter(goal),TranslateRule());
  
  AnswerSet answer = reasoner->computePlan(goal);
  
  res.plan.satisfied = answer.isSatisfied();
  transform(answer.getFluents().begin(),answer.getFluents().end(),back_inserter(res.plan.fluents),TranslateFluent());
  
  return true;
}