// -*- mode: C++ -*-
// -*- c-file-style: bsd -*-

///
//   Make two plans concurrently.

#ifndef _PLAN_CONCURRENTLY_H_
#define _PLAN_CONCURRENTLY_H_ 1

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include "actions/Action.h"


/// Run two planning functions concurrently
//
//  @param fn1 first callable object, returning std::list<T>
//  @param fn2 second callable object, returning std::list<T>
//
//  @return the first available non-empty list result.  If both are
//          empty, planning has failed.
//
//  This is a stub for testing, it does not run fn1 and fn2 in parallel.
//
template <typename T, class F1, class F2>
std::list<T> plan_concurrently(F1 fn1, F2 fn2) {

#if 1   // serial implementation

        std::list<T> result = fn1();
        if (result.empty())
                result = fn2();
        return result;

#else   // parallel implementation

        // This implementation is rather crude, but I can't find a
        // standard solution.  It is the simplest I could think of
        // that might work.
        //
        // Creating and destroying two threads on each call is
        // wasteful, but that cost is tiny compared to forking and
        // execing a shell and a clasp process to compute each plan.

        // Internal class for wrapping a function in a thread
        template <typename T, class F>
        class _ThreadWrapper {
        public:
                std::list<T> result;
                boost::thread thr;
        
                _ThreadWrapper():
                        thr(_ThreadWrapper::tfn, this, F) {}
                void tfn(F func) {result = func();}
                bool finished() const {return thr.joinable;}
        };

        _ThreadWrapper<T, F1> t1;
        _ThreadWrapper<T, F2> t2;

        // Polling for thread completion is absurd, but adequate for
        // our needs.  Feel free to make it better, if necessary.
        while (true) {
                // If both finish together, we prefer the second plan.
                if (t2.finished() && !t2.result.empty())
                        // second function returned a plan
                        return t2.result;
                else if (t1.finished() && !t1.result.empty())
                        // first function returned a plan
                        return t1.result;
                else if (t1.finished() && t2.finished())
                        // both are finished and empty: failure
                        return t1.result;
                else
                        usleep(1000);   // wait a millisecond
        }
#endif
}

#endif // _PLAN_CONCURRENTLY_H_
