// -*- mode: C++ -*-
// -*- c-file-style: bsd -*-

///
//   Make two plans concurrently.

#ifndef _PLAN_CONCURRENTLY_H_
#define _PLAN_CONCURRENTLY_H_ 1

#include "actions/Action.h"
#include "action_utils.h"

#include <boost/thread/thread.hpp>

namespace bwi_actexec {

struct Operation {
	virtual void operator()() = 0;
	
	virtual bool finished() const = 0;
	
	virtual std::list<Action *> computedPlan() = 0;
	
	virtual ~Operation(){}
};


struct Replan : public Operation {
	
	Replan(const std::string& goal, int max_n);
	
	void operator()();
	
	bool finished() const {return done;}
	
	std::list<Action *> computedPlan() {copied = true; return finalPlan;}
	
	~Replan();
	
	int max_n;
	const std::string goalSpecification;
	std::list<Action *> finalPlan;
	bool done;
	bool copied;
};


struct Repair : public Operation {
	
	Repair(const std::list<Action *>& plan,const std::string& goal, unsigned int max_changes);

	void operator()();
	
	bool finished() const {return done;}
	
	std::list<Action *> computedPlan() {copied = true; return finalPlan;}
	
	~Repair();
	
	std::list<Action *> plan;
    std::string goalSpecification;
    unsigned int max_changes;
	std::list<Action *> finalPlan;
	bool done;
	bool copied;
};

}


#endif // _PLAN_CONCURRENTLY_H_
