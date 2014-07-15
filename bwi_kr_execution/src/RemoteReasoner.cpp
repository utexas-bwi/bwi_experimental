#include "RemoteReasoner.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include "bwi_kr_execution/UpdateFluents.h"
#include "bwi_kr_execution/ComputePlan.h"
#include "bwi_kr_execution/ComputeAllPlans.h"
#include "bwi_kr_execution/IsPlanValid.h"

#include "msgs_utils.h"

#include "actasp/Action.h"

#include <ros/ros.h>

#include <algorithm>
#include <iterator>

using namespace std;
using namespace ros;

namespace bwi_krexec {
  
actasp::AnswerSet RemoteReasoner::currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() {
  NodeHandle n;
  ros::ServiceClient currentClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
  currentClient.waitForExistence();
  
  bwi_kr_execution::CurrentStateQuery csq;
  
  transform(query.begin(),query.end(),back_inserter(csq.request.query),TranslateRule());
  
  currentClient.call(csq);
  
  return TranslateAnswerSet()(csq.response.answer);
}

bool RemoteReasoner::updateFluents(const std::vector<actasp::AspFluent> &observations) throw() {
  NodeHandle n;
  ros::ServiceClient updateClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
  updateClient.waitForExistence();
  
  bwi_kr_execution::UpdateFluents uf;
  
  transform(observations.begin(),observations.end(),back_inserter(uf.request.fluents),TranslateFluent());
  
  updateClient.call(uf);
  
  return uf.response.consistent;
}

actasp::AnswerSet RemoteReasoner::computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error) {
  
  NodeHandle n;
  ros::ServiceClient planClient = n.serviceClient<bwi_kr_execution::ComputePlan> ( "compute_plan" );
  planClient.waitForExistence();
  
  bwi_kr_execution::ComputePlan cp;
  
  transform(goal.begin(),goal.end(),back_inserter(cp.request.goal),TranslateRule());
  
  planClient.call(cp);
  
  return TranslateAnswerSet()(cp.response.plan);
  
}

actasp::AnswerSet RemoteReasoner::computePlan(const std::vector<actasp::AspRule>& goal) throw (std::logic_error) {
  const RemoteReasoner *constThis = this;
  return constThis->computePlan(goal);
}

struct ActionToFluent {
  ActionToFluent() : counter(0) {}
  actasp::AspFluent operator()(const actasp::Action *act) {
    return actasp::AspFluent(act->toASP(counter++));
  }
  int counter;
};

bool RemoteReasoner::isPlanValid(std::list<actasp::Action*> plan, const std::vector<actasp::AspRule>& goal)  const throw() {
  
  vector<actasp::AspFluent> fluentVector;
  transform(plan.begin(),plan.end(),back_inserter(fluentVector),ActionToFluent());
  
  actasp::AnswerSet answerSetPlan(true,fluentVector);
  
  return this->isPlanValid(answerSetPlan,goal);
}


bool RemoteReasoner::isPlanValid(actasp::AnswerSet plan, const std::vector<actasp::AspRule>& goal)  const throw() {
  NodeHandle n;
  ros::ServiceClient planClient = n.serviceClient<bwi_kr_execution::IsPlanValid> ( "is_plan_valid" );
  planClient.waitForExistence();
  
  bwi_kr_execution::IsPlanValid ipv;
  
  vector<bwi_kr_execution::AspRule> goalRules;
  transform(goal.begin(),goal.end(),back_inserter(goalRules),TranslateRule());
  ipv.request.goal = goalRules;
  
  transform(plan.getFluents().begin(),plan.getFluents().end(),back_inserter(ipv.request.plan.fluents),TranslateFluent());
  ipv.request.plan.satisfied = plan.isSatisfied();
  
  planClient.call(ipv);
  
  return ipv.response.valid;
  
}

std::vector< actasp::AnswerSet > RemoteReasoner::computeAllPlans(
                const std::vector<actasp::AspRule>& goal, 
                double suboptimality) throw (std::logic_error) {
  NodeHandle n;
  ros::ServiceClient planClient = n.serviceClient<bwi_kr_execution::ComputeAllPlans> ( "compute_all_plans" );
  planClient.waitForExistence();
  
  bwi_kr_execution::ComputeAllPlans cap;
  
  transform(goal.begin(),goal.end(),back_inserter(cap.request.goal),TranslateRule());
  
  cap.request.suboptimality = suboptimality;
  
  planClient.call(cap);
  
  vector<actasp::AnswerSet> computedPlans;
  transform(cap.response.plans.begin(),cap.response.plans.end(),back_inserter(computedPlans),TranslateAnswerSet());
  
  return computedPlans;
  
                  
}

actasp::MultiPolicy RemoteReasoner::computePolicy(const std::vector<actasp::AspRule>& goal, 
                                  double suboptimality) throw (std::logic_error) {
 ROS_ERROR ("RemoteReasoner: computePolicy is not implemented");
 return actasp::MultiPolicy();
}

}
