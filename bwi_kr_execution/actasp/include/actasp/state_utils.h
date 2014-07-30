

#ifndef actasp_state_util_h__guard
#define actasp_state_util_h__guard

#include <actasp/AspFluent.h>

#include <functional>
#include <set>

namespace actasp {

struct StateComparator : public std::binary_function<const std::set<AspFluent>&,const std::set<AspFluent>&, bool> {
    
  bool operator()(const std::set<AspFluent> &first,const std::set<AspFluent> &second) const;
    
};

}

#endif
