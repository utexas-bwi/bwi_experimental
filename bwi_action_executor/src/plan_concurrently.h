// -*- mode: C++ -*-
// -*- c-file-style: bsd -*-

///
//   Make two plans concurrently.

#ifndef _PLAN_CONCURRENTLY_H_
#define _PLAN_CONCURRENTLY_H_ 1

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include "actions/Action.h"

class PlanConcurrently {

public:

        typedef std::list<bwi_actexec::Action *> Plan;

        PlanConcurrently() {}
        ~PlanConcurrently() {}

        /// make plans concurrently
        //
        //  @param f1 callable object for first plan
        //  @param f2 callable object for second plan
        //
        //  @return the first available non-empty list of actions.  If
        //          both are empty planning has failed.
        //
        //  This is a stub for testing, it always fails.
        template <class F1, class F2>
        Plan plan(F1 fn1, F2 fn2) {
                return Plan();          // empty plan
        }
};

#endif // _PLAN_CONCURRENTLY_H_
