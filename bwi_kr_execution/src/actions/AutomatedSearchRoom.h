
#ifndef bwi_krexec_AutomatedSearchRoom_h__guard
#define bwi_krexec_AutomatedSearchRoom_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>

#include <sound_play/SoundRequest.h>

#include <string>

namespace bwi_krexec {

class AutomatedSearchRoom : public actasp::Action{
public:
  AutomatedSearchRoom();

  int paramNumber() const {return 2;}
  
  std::string getName() const {return "searchroom";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new AutomatedSearchRoom(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string person;
 std::string room;
 bool done;

 bool started;
 float totalTime;
 ros::Time starTime;
 
 static std::map<std::string, std::string> person_location_map;
 static bool person_location_available;
};

}
 
#endif
 
