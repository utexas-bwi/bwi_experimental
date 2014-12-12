#include <actasp/AspFluent.h>

#include <sstream>
#include <cstdlib> //for atoi

using namespace std;

namespace actasp {

static vector<string> tokenizeCommas(string input) {

	vector<string> stuff;

	while (!input.empty()) {

		int firstComma = input.find(",");
		stuff.push_back(input.substr(0, std::min(firstComma, (int) input.length())));

		if (firstComma == string::npos)
			input = "";
		else
			input = input.substr(firstComma+1);
	}
	return stuff;
}


AspFluent::AspFluent(const std::string& formula) throw (std::invalid_argument) :
      name(),
      variables(),
      timeStep(),
			cachedBase(){
        
  //this used to be nice, but it turned out to be a major bottleneck, so I had to reimplement it for efficiency.
   
   bool inName = true;   
   bool valid = false;
   string current;
   current.reserve(100);
   cachedBase.reserve(100);
   
   for(int i=0, size = formula.length(); i < size; ++i) {
     
     if(inName) {
       if(formula.at(i) != '(')
         name += formula.at(i);
       else {
         cachedBase += name;
         cachedBase += '(';
         inName = false;
       }
     }
     else {
       if(formula.at(i) == ')') {
         timeStep = atoi(current.c_str());

           
         valid = true;
         break; //ignore anything else
       }
       if(formula.at(i) != ',')
         current += formula.at(i);
       else {
         variables.push_back(current);
         cachedBase += current;
         cachedBase += ',';
         current.clear();
       }
     }
   }

   if(inName)
     throw std::invalid_argument("AspFluent: The string " + formula + " does not contain a '(', therefore is not a valid fluent");
   
   if(!valid)
     throw std::invalid_argument("The string " + formula + " does not contain a ')', therefore is not a valid fluent");
}

AspFluent::AspFluent(const std::string &name, const std::vector<std::string> &variables, unsigned int timeStep) throw () : 
		name(name),
		variables(variables),
		timeStep(timeStep),
		cachedBase() {
  stringstream ss;

  ss << name << "(";

  int i=0;
  for (int size = variables.size(); i<size; ++i)
    ss << variables[i] << ",";

  cachedBase = ss.str();

}

unsigned int AspFluent::arity() const  throw() {
	return this->variables.size() + 1;
}

void AspFluent::setTimeStep(unsigned int timeStep) throw() {
	

	this->timeStep = timeStep;
	
}

unsigned int AspFluent::getTimeStep() const throw() {
	return this->timeStep;
}

string AspFluent::getName() const throw() {
	return name;
}

vector<string> AspFluent::getParameters() const throw() {
	return variables;
}

bool AspFluent::operator<(const AspFluent& other) const throw(){
	if(this->timeStep < other.timeStep)
		return true;

	if(this->timeStep > other.timeStep)
		return false;

	return  this->cachedBase < other.cachedBase;
}

bool AspFluent::operator==(const AspFluent& other) const throw() {
	if(this->timeStep != other.timeStep)
		return false;
	
	return this->cachedBase == other.cachedBase;
}

std::string AspFluent::toString(unsigned int timeStep) const throw() {
    
  stringstream ss;
  ss << timeStep << ")";
  return cachedBase + ss.str();
}

std::string AspFluent::toString(const string& timeStepVar) const throw() {   
  return cachedBase + timeStepVar + ")";
}

std::string AspFluent::toString() const throw () {
	return this->toString(this->timeStep);
}


}