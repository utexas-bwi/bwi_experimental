
#ifndef bwi_krexec_RemoteReasoner_h__guard
#define bwi_krexec_RemoteReasoner_h__guard

#include "actasp/AspKR.h"

namespace bwi_krexec {
  
  class RemoteReasoner : public actasp::AspKR {
  public:
  
  actasp::AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
  
  bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();
  
  bool isPlanValid(const actasp::AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw();
  
  std::vector< actasp::AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) throw (std::logic_error);

  actasp::MultiPolicy computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) throw (std::logic_error);
  
  actasp::AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) throw (std::logic_error);
  actasp::AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error);
  
  };
  
}

#endif
