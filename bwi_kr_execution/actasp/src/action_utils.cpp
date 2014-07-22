#include <actasp/action_utils.h>

using namespace std;

namespace actasp {
  
AnswerSet planToAnswerSet(const std::list<Action*>& plan) {
  
  list<Action*>::const_iterator actIt = plan.begin();
  vector<AspFluent> fluents;
  fluents.reserve(plan.size());
  
  for(int timeStep=0; actIt != plan.end(); ++actIt, ++timeStep)
    fluents.push_back((*actIt)->toFluent(timeStep));
  
  return AnswerSet(true,fluents);
}

std::vector<AspFluent> actionMapToVector(const std::map<std::string, Action *>& actionMap) {
  
  vector<AspFluent> fluents;
  fluents.reserve(actionMap.size());
  
  std::map<std::string, Action *>::const_iterator mapIt = actionMap.begin();
  for(; mapIt != actionMap.end(); ++mapIt)
    fluents.push_back(AspFluent(mapIt->second->toFluent(0)));
  
  return fluents;
}

}
