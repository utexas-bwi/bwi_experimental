
#ifndef actasp_execution_observer_utiles_h__guard
#define actasp_execution_observer_utiles_h__guard

#include <actasp/AspFluent.h>
#include <actasp/ExecutionObserver.h>

namespace actasp {

  
struct NotifyActionTermination {
  
  NotifyActionTermination(const AspFluent& action, bool failed) : action(action), failed(failed) {}
  
  void operator()(ExecutionObserver *observer) {
    observer->actionTerminated(action, failed);
  }
  
  AspFluent action;
  bool failed;
};

struct NotifyActionStart {
  
  NotifyActionStart(const AspFluent& action, const std::set<AspFluent>& state) : 
    action(action), startState(state) {}
  
  void operator()(ExecutionObserver *observer) {
    observer->actionStarted(action, startState);
  }
  
  AspFluent action;
  std::set<AspFluent> startState;
};

struct NotifyPlanExecutionFailed {
  
  void operator()(ExecutionObserver *observer) {
    observer->planExecutionFailed();
  }

};

struct NotifyPlanExecutionSucceeded {
  
  void operator()(ExecutionObserver *observer) {
    observer->planExecutionSucceeded();
  }

};

}

#endif
