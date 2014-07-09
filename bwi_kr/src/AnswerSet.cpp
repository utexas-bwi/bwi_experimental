#include <bwi_kr/AnswerSet.h>

#include <sstream>
#include <iterator>

using namespace std;

namespace bwi_kr {

static	vector<string> tokenizeCommas(string input);
	
AnswerSet::AnswerSet(const std::string& answer) {
	
	
	satisfiable = answer.find("UNSATISFIABLE") == string::npos;
	
	if(satisfiable) {
	
		stringstream content(answer);
		
		string line;
		getline(content,line);
		getline(content,line);
		
		stringstream predicateLine(line);
		
		vector<string> predicateStrings;
		
		copy(istream_iterator<string>(predicateLine),
              istream_iterator<string>(),
              back_inserter<vector<string> >(predicateStrings));
		
		
		vector<string>::const_iterator predIt = predicateStrings.begin();
		
		for(; predIt != predicateStrings.end(); ++predIt) {
			
			Predicate pred;

			int start = predIt->find("(");

			pred.name = predIt->substr(0,start);
			
			int end = predIt->find_last_of(",");
			string paramString = predIt->substr(start+1,end-(start+1));

			pred.parameters = tokenizeCommas(paramString);
		
			start = predIt->find_last_of(",") +1;
			end = predIt->find_last_of(")");
			pred.timeStep = atoi((predIt->substr(start,end-start)).c_str());
			
			predicates.push_back(pred);
			
		}
	}
	
}

vector<string> tokenizeCommas(string input) {
	
	vector<string> stuff;

	while (!input.empty()) {

		int firstComma = input.find(",");
		stuff.push_back(input.substr(0, std::min(firstComma, (int) input.length())));

		if(firstComma == string::npos)
			input = "";
		else
			input = input.substr(firstComma+1);
	}
	return stuff;
}


}
