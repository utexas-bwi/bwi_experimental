
#ifndef bwi_krexec_RemoteReasoner_h__guard
#define bwi_krexec_RemoteReasoner_h__guard

#include "actasp/AspKR.h"
#include "actasp/reasoners/Clingo.h"

namespace bwi_krexec {
  
  class RemoteReasoner : public actasp::AspKR {
  public:
    
    RemoteReasoner(unsigned int max_n,
         const std::string& queryDir,
         const std::string& domainDir,
         const actasp::ActionSet& actions);
  
  actasp::AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
  
  bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();
  
  bool isPlanValid(const actasp::AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw();
  
  std::vector< actasp::AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);
  
  void reset() throw();

  actasp::MultiPolicy computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);
  
  actasp::AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error);
  
  private:
    actasp::Clingo local;
  
  };
}

#endif
