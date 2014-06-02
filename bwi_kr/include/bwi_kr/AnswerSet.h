#ifndef bwi_kr_AnswerSet_h__guard 
#define bwi_kr_AnswerSet_h__guard

#include <string>
#include <vector>

namespace bwi_kr {

class AnswerSet {
public:
	AnswerSet(const std::string& answer);
	
	bool isSatisfied() { return satisfiable;}
	
	std::vector<std::string> getPredicates() {return predicates;}
	
private:
	bool satisfiable;
	std::vector<std::string> predicates;
};



}

#endif
