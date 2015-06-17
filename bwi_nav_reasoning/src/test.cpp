

#include <boost/foreach.hpp>
#include <bwi_rl/planning/PredictiveModel.h>
#include <bwi_rl/planning/ValueIteration.h>
#include <bwi_rl/planning/VITabularEstimator.h>

#include "NavMdp.h"


std::ostream& operator<<(std::ostream& stream, const Action& action) {
  if (action == UP) {
    stream << "Up";
  } else if (action == DOWN) {
    stream << "Down";
  } else if (action == LEFT) {
    stream << "Left";
  } else {
    stream << "Right";
  }
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const State& s) {
  stream << "(" << s.x << "," << s.y << ")";
  return stream;
}

int main(int argc, char **argv) {

    boost::shared_ptr<PredictiveModel<State, Action> > model (new NavMdp(
        "/gdc_atrium/static_map.txt", "/gdc_atrium/dynamic_map.txt", 
        "/gdc_atrium/sunny_map.txt", "/tmp/rl_domain/facts.plog"); 
    boost::shared_ptr<VIEstimator<State, Action> > estimator(new
        VITabularEstimator<State, Action>); 

    ValueIteration<State, Action> vi(model, estimator);
    std::cout << "Computing policy..." << std::endl;
    vi.computePolicy(); 

    State s = State(0, 0); // top-left conner
    std::cout << "best action at state " << s << " is " << vi.getBestAction(s) << std::endl; 

    return 0; 
}
