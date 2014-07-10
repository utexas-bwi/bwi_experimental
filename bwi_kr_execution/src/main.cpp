
#include "actasp/reasoners/Clingo.h"

#include "actions/ActionFactory.h"

#include <ros/ros.h>

#include <boost/filesystem.hpp>

#include <string>

const int MAX_N = 40;
const std::string queryDirectory("/tmp/bwi_kr_execution/"); 

using namespace std;
using namespace bwi_krexec;

int main(int argc, char**argv) {
	
	const string domainDirectory("domain_simulation"); //TODO get this from a parameter
	ActionFactory::setSimulation(true);
	
	
	boost::filesystem::create_directories(queryDirectory);
	actasp::Clingo clingo(MAX_N,queryDirectory,domainDirectory,ActionFactory::actions());
	
	
	
	
return 0;	
}