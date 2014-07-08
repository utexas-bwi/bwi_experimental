
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
	       const std::map<std::string, actasp::Action*>& actionMap) throw();

	AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
	
	bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();

	bool isPlanValid(std::list<actasp::Action*> plan, const std::vector<actasp::AspRule>& goal) const throw();

	std::list<Action *> computePlan(const std::vector<actasp::AspRule>& goal) throw (std::logic_error);
	
	std::vector< std::list<Action *> > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) throw (std::logic_error);
	
	MultiPolicy computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) throw (std::logic_error);
	
	void setMaxTimeStep(unsigned int max_n) throw() {
		this->max_n = max_n;
	}

	~Clingo();

private:

	unsigned int max_n;
	std::string queryDir;
	std::string domainDir;
	std::map<std::string, Action * > actionMap;

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
