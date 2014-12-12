#ifndef actasp_AspFluent_h__guard
#define actasp_AspFluent_h__guard

#include <string>
#include <vector>
#include <stdexcept>
#include <set>
#include <functional>

namespace actasp {

class ActionComparator;
class ActionEquality;
  
class AspFluent {
public:

	AspFluent(const std::string& formula) throw (std::invalid_argument);
	AspFluent(const std::string &name, const std::vector<std::string> &variables, unsigned int timeStep = 0) throw ();

	unsigned int arity() const throw ();

	void setTimeStep(unsigned int timeStep) throw();
	unsigned int getTimeStep() const throw();
	
	std::string getName() const throw();
	std::vector<std::string> getParameters() const throw();
	
	bool operator<(const AspFluent& other) const throw();
	bool operator==(const AspFluent& other) const throw();

	std::string toString() const throw();
	std::string toString(unsigned int timeStep) const throw();
  std::string toString(const std::string& timeStepVar) const throw();
  
  operator std::string() const { return this->toString(); } 

private:
	std::string name;
	std::vector<std::string> variables;
	unsigned int timeStep;
	std::string cachedBase; //cached for optimization
	
	friend class ActionComparator;
  friend class ActionEquality;

};

struct ActionComparator : public std::binary_function<const AspFluent&, const AspFluent&, bool>{
 bool operator()(const AspFluent& first, const AspFluent& second) const {
   return first.cachedBase < second.cachedBase;
 }
};

struct ActionEquality : public std::binary_function<const AspFluent&, const AspFluent&, bool>{
 bool operator()(const AspFluent& first, const AspFluent& second) const {
   return first.cachedBase == second.cachedBase;
 }
};

typedef std::set<AspFluent, ActionComparator> ActionSet;

}
#endif
