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
			cachedString(){

	size_t start = formula.find("(");

	if (start == string::npos)
		throw std::invalid_argument("AspFluent: The string " + formula + " does not contain a '(', therefore is not a valid fluent");

	this->name = formula.substr(0,start);

	size_t end = formula.find_last_of(",");

	//if there is no comma, the action has only one parameter, which must be the time step
	string paramString = (end == string::npos)? "" : formula.substr(start+1,end-(start+1));

	this->variables = tokenizeCommas(paramString);

	//if there is no comma, consider the parenthesis
	if(end != string::npos)
		start = end + 1;
	else
		start = start + 1;
	
	end = formula.find_last_of(")");

	if (end == string::npos)
		throw std::invalid_argument("The string " + formula + " does not contain a ')', therefore is not a valid fluent");

	//assumes zero if it fails
	timeStep = atoi((formula.substr(start,end-start)).c_str());
	
	cachedString = this->toString(this->timeStep);
}

AspFluent::AspFluent(const std::string &name, const std::vector<std::string> &variables, unsigned int timeStep) throw () : 
		name(name),
		variables(variables),
		timeStep(timeStep),
		cachedString(){
			cachedString = this->toString(this->timeStep);
		}

unsigned int AspFluent::arity() const  throw() {
	return this->variables.size() + 1;
}

void AspFluent::setTimeStep(unsigned int timeStep) throw() {
	
	//may invalidate the cached string
	
	if(timeStep != this->timeStep)
		cachedString = toString(timeStep);
	
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

	return  this->toString() < other.toString();
}

bool AspFluent::operator==(const AspFluent& other) const throw() {
	if(this->timeStep != other.timeStep)
		return false;
	
	return this->toString() == other.toString();
}

std::string AspFluent::toString(unsigned int timeStep) const throw() {
	
	if(timeStep == this->timeStep && !cachedString.empty())
		return cachedString;

	stringstream ss;

	ss << name << "(";

	int i=0;
	for (int size = variables.size(); i<size; ++i)
		ss << variables[i] << ",";

	ss << timeStep << ")";

	return ss.str();

}

std::string AspFluent::toString() const throw () {
	return cachedString;
}


}