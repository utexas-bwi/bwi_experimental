
#include <actasp/state_utils.h>

using namespace std;

namespace actasp {


bool  StateComparator::operator()(const std::set<AspFluent> &first,const std::set<AspFluent> &second) const {
  if (first.size() != second.size())
    return first.size() < second.size();

  //they have the same number of fluents
  set<AspFluent>::const_iterator thisV = first.begin();
  set<AspFluent>::const_iterator otherV = second.begin();

  ActionComparator lessThen; //ignores the time steps!

  for (; thisV != first.end(); ++thisV, ++otherV) {
    //this comparison is costly, so I'm using this unelegant expression to minimize the calls to it.
    if (lessThen(*thisV , *otherV))
      return true;
    if (lessThen(*otherV , *thisV))
      return false;
  }

  //they are the same
  return false;
}


}

