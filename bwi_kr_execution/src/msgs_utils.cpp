
#include "msgs_utils.h"

#include <algorithm>
#include <iterator>

using namespace std;

namespace bwi_krexec {

actasp::AspFluent TranslateFluent::operator()(const bwi_kr_execution::AspFluent& bwiFluent) {
   return actasp::AspFluent(bwiFluent.name,bwiFluent.variables,bwiFluent.timeStep);
}

bwi_kr_execution::AspFluent TranslateFluent::operator()(const actasp::AspFluent& actaspFluent) {
  
  bwi_kr_execution::AspFluent bwiFluent;
  bwiFluent.name = actaspFluent.getName();
  bwiFluent.timeStep = actaspFluent.getTimeStep();
  bwiFluent.variables = actaspFluent.getParameters();
  
  return bwiFluent;
}


actasp::AspRule TranslateRule::operator()(const bwi_kr_execution::AspRule& bwiRule) {
  actasp::AspRule aspRule;
  
  transform(bwiRule.head.begin(), bwiRule.head.end(), back_inserter(aspRule.head), TranslateFluent());
  transform(bwiRule.body.begin(), bwiRule.body.end(), back_inserter(aspRule.body), TranslateFluent());

  
  return aspRule;
}

actasp::AnswerSet TranslateAnswerSet::operator()(const bwi_kr_execution::AnswerSet& bwiAnswerSet) {
 vector<actasp::AspFluent> fluents;
 transform(bwiAnswerSet.fluents.begin(), bwiAnswerSet.fluents.end(), back_inserter(fluents), TranslateFluent());
 return actasp::AnswerSet(bwiAnswerSet.satisfied, fluents);
}



}
