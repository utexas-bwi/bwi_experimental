#ifndef bwi_krexec_ChangeFloor_h__guard
#define bwi_krexec_ChangeFloor_h__guard

#include "actasp/Action.h"


namespace bwi_krexec {
  
struct ChangeFloor : public actasp::Action {
    
  int paramNumber() const {return 1;}
  
  std::string getName() const{return "changefloor";}
  
  void run();
  
  bool hasFinished() const;
  
  bool hasFailed() const;
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  actasp::Action *clone() const {return new ChangeFloor(*this);}

private:
 
std::vector<std::string> getParameters() const;

std::string dest_room;

};
  
  
}

#endif
