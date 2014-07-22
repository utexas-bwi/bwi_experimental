
#ifndef actasp_Clingo_h__guard
#define actasp_Clingo_h__guard

#include <actasp/AspKR.h>
#include <actasp/MultiPolicy.h>

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>

namespace actasp {

class Action;

class Clingo : public AspKR {
public:

	Clingo(unsigned int max_n,
	       const std::string& queryDir,
	       const std::string& domainDir,
	       const std::vector<AspFluent>& actions) throw();

	AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
	
	bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();
	
	bool isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw();

	AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) throw ();
	
	std::vector< AnswerSet> computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) throw ();
	
	MultiPolicy computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) throw (std::logic_error);
	
	void setMaxTimeStep(unsigned int max_n) throw() {
		this->max_n = max_n;
  }

private:

	unsigned int max_n;
	std::string queryDir;
	std::string domainDir;
  std::vector<AspFluent> allActions;

	std::string generatePlanQuery(	const std::vector<actasp::AspRule>& goalRules, 
									unsigned int timeStep, 
									bool filterActions) const throw();

	std::vector<actasp::AnswerSet> krQuery(	const std::string& query, 
											unsigned int timeStep,
											const std::string& fileName, 
											unsigned int answerSetsNumber) const throw();
                      

};

}
#endif
