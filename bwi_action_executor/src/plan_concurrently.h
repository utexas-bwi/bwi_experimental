// -*- mode: C++ -*-
// -*- c-file-style: bsd -*-

///
//   Make two plans concurrently.

#ifndef _PLAN_CONCURRENTLY_H_
#define _PLAN_CONCURRENTLY_H_ 1

#include <unistd.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

namespace detail {

/// Internal class for wrapping a function in a thread
//
//  @param func callable object of type F, returning std::list<T>
//
//  @post Thread started, which calls *func*.
//  @post When *func* returns, this->finished() is true and *result*
//        contains the function value.
//
template <typename T, class F>
class _ThreadWrapper {
public:
        std::list<T> result;
        boost::thread thr;

        _ThreadWrapper(F func):
                detached_(false),
                running_(true),
                success_(false),
                terminate_(false),
                func_ (func),
                thr (boost::thread(&_ThreadWrapper::tfn_, this)) {}
        void detach() {
                detached_ = true;
                if (running_)
                        // tell thread to delete itself
                        thr.interrupt();
                else
                        // clean up thread memory
                        delete this;
        }
        bool successful() {return (!running_ && success_);}
        bool failed() {return (!running_ && !success_);}
        bool finished() {return !thr.joinable();}
        //void terminate() const {
        //        thr.interrupt();
        //        terminate_ = true;
        //}

private:
        F func_;
        volatile bool detached_;
        volatile bool running_;
        volatile bool success_;
        volatile bool terminate_;
        void tfn_() {
                result = func_();
                if (!result.empty())
                        success_ = true;
                // clean up memory if result no longer needed
                if (detached_)
                        delete this;
                else
                        running_ = false;
        }
};


/// Internal class for allocating a new thread instance.
//
//  @param func callable object of type F, returning std::list<T>
//
//  Allocates and starts a thread.  The destructor leaves the
//  _ThreadWrapper object allocated, to delete itself when
//  interrupted.  Acts like a pointer to a _ThreadWrapper.
//
template <typename T, class F>
class _AllocThread {
public:
        _AllocThread(F func):
                wrapper_(new _ThreadWrapper<T, F>(func)) {}
        ~_AllocThread() {wrapper_->detach();}
        _ThreadWrapper<T, F> * operator->() const {return wrapper_;}
        _ThreadWrapper<T, F> & operator*() const {return *wrapper_;}
private:
        _ThreadWrapper<T, F> *wrapper_;
};
};  // namespace detail

/// Run two planning functions concurrently
//
//  @param fn1 first callable object, returning std::list<T>
//  @param fn2 second callable object, returning std::list<T>
//
//  @return the first available non-empty list result.  If both are
//          empty, planning has failed.
//
template <typename T, class F1, class F2>
std::list<T> plan_concurrently(F1 fn1, F2 fn2) {

#if 1   // parallel implementation

        // This implementation is rather crude, but I can't find a
        // standard solution.  It is the simplest I could think of
        // that might work.
        //
        // Creating and destroying two threads on each call is
        // wasteful, but that cost is tiny compared to forking and
        // execing a shell and a clasp process to compute each plan.

        detail::_AllocThread<T, F1> t1(fn1);
        detail::_AllocThread<T, F2> t2(fn2);

        // Polling for thread completion is absurd, but adequate for
        // our needs.  Feel free to make it better, if necessary.
        std::list<T> result;
        while (true) {
                // If both finish together, we prefer the second plan.
                if (t2->successful()) {
                        // second function returned a plan
                        result = t2->result;
                        break;
                }
                if (t1->successful()) {
                        // first function returned a plan
                        result = t1->result;
                        break;
                }
                if (t1->failed() && t2->failed()) {
                        // both are finished now
                        result = t2->result;
                        break;
                }
                usleep(1000);           // wait a millisecond
        }

        return result;

#else   // serial implementation

        std::list<T> result = fn1();
        if (result.empty())
                result = fn2();
        return result;
#endif
}

#endif // _PLAN_CONCURRENTLY_H_
