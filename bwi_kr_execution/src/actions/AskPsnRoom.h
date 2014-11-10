
#ifndef bwi_krexec_AskPsnRoom_h__guard
#define bwi_krexec_AskPsnRoom_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>

#include <sound_play/SoundRequest.h>

#include <string>

namespace bwi_krexec {

class AskPsnRoom : public actasp::Action{
public:
  AskPsnRoom();

  int paramNumber() const {return 2;}
  
  std::string getName() const {return "askpsnroom";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new AskPsnRoom(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string person_to_ask;
 std::string person_to_know;
 static ros::Publisher ask_pub;
 static bool pub_set;
 bool done;
 
};

}
 
#endif
 