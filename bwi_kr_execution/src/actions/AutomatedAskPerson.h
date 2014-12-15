
#ifndef bwi_krexec_AutomatedAskPerson_h__guard
#define bwi_krexec_AutomatedAskPerson_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>

#include <sound_play/SoundRequest.h>

#include <string>

namespace bwi_krexec {

class AutomatedAskPerson : public actasp::Action {
public:
  AutomatedAskPerson();

  int paramNumber() const {return 2;}
  
  std::string getName() const {return "askperson";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new AutomatedAskPerson(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string person_to_ask;
 std::string person_to_know;
 bool done;
 
 bool started;
 ros::Time starTime;
 
 static std::map<std::string, std::string> person_location_map;
 static bool person_location_available;
};

}
 
#endif
 
