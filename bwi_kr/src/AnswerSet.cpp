#include <bwi_kr/AnswerSet.h>

#include <sstream>
#include <iterator>

using namespace std;

namespace bwi_kr {

AnswerSet::AnswerSet(const std::string& answer) {
	
	
	satisfiable = answer.find("UNSATISFIABLE") == string::npos;
	
	if(satisfiable) {
	
		stringstream content(answer);
		
		string line;
		getline(content,line);
		getline(content,line);
		
		stringstream predicateLine(line);
		
		copy(istream_iterator<string>(predicateLine),
              istream_iterator<string>(),
              back_inserter<vector<string> >(predicates));
	}
	
}


}