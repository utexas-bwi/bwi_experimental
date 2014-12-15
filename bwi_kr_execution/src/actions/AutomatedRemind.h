#ifndef bwi_krexec_AutomatedRemind_h__guard
#define bwi_krexec_AutomatedRemind_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>
#include <string>

namespace bwi_krexec {

class AutomatedRemind : public actasp::Action{
public:
  AutomatedRemind();

  int paramNumber() const {return 3;}
  
  std::string getName() const {return "remind";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new AutomatedRemind(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string person_to_remind;
 std::string meeting;
 std::string room;

 bool done;

 bool started;
 ros::Time starTime;
 
};

}
 
#endif
 
