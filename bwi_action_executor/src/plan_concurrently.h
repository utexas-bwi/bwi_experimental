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
        std::list<T> result = fn1();
        if (result.empty())
                result = fn2();
        return result;
}

#endif // _PLAN_CONCURRENTLY_H_
