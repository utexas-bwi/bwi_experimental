#ifndef bwi_kr_AnswerSet_h__guard 
#define bwi_kr_AnswerSet_h__guard

#include <bwi_kr/Predicate.h>

#include <string>
#include <vector>

namespace bwi_kr {

class AnswerSet {
public:
	AnswerSet(const std::string& answer);
	
	bool isSatisfied() { return satisfiable;}
	
	std::vector<Predicate> getPredicates() {return predicates;}
	
private:
	bool satisfiable;
	std::vector<Predicate> predicates;
};



}

#endif
