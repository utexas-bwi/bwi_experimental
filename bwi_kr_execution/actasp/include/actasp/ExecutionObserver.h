#ifndef actasp_ExecutionObserver_h__guard
#define actasp_ExecutionObserver_h__guard



namespace actasp {

class AspFluent;
class AnswerSet;

struct ExecutionObserver {

  virtual void actionStarted(const AspFluent& action, const std::set<AspFluent>& current_state) throw() =0 ;
  virtual void actionTerminated(const AspFluent& action, bool failed) throw() =0;

  virtual void planExecutionFailed() throw() = 0;
  virtual void planExecutionSucceeded() throw() = 0;

  virtual ~ExecutionObserver() {}
};

}


#endif
