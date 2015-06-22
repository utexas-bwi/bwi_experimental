#include <boost/foreach.hpp>
#include <iostream>
#include <bwi_rl/planning/PredictiveModel.h>
#include <bwi_rl/planning/ValueIteration.h>
#include <bwi_rl/planning/VITabularEstimator.h>

#include "bwi_nav_reasoning/NavMdp.h"
#include "bwi_nav_reasoning/StateAction.h"
#include "bwi_nav_reasoning/DomainParser.h"

bool operator<(const State& l, const State& r) {
  if (l.row < r.row) return true;
  if (l.row > r.row) return false;

  if (l.col < r.col) return true;
  if (l.col > r.col) return false;
}
                                                                                
bool operator==(const State& l, const State& r) {                               
  return (l.row == r.row && l.col == r.col);                                            
}                                                                               

int main(int argc, char **argv) {

    std::cout << "creating nav model..." << std::endl; 
    boost::shared_ptr<PredictiveModel<State, Action> > model (new NavMdp(
        "/gdc_atrium/static_map.txt", "/gdc_atrium/dynamic_map.txt", 
        "/gdc_atrium/sunny_map.txt", "/tmp/rl_domain/facts.plog")); 

    std::cout << "creating vi estimator..." << std::endl; 
    boost::shared_ptr<VIEstimator<State, Action> > estimator(new
        VITabularEstimator<State, Action>); 

    ValueIteration<State, Action> vi(model, estimator);
    std::cout << "Computing policy..." << std::endl;
    vi.computePolicy(); 

    State s, terminal; 
    s.row = 2;
    s.col = 3; 
    terminal.row = 0; 
    terminal.col = 3; 
    boost::shared_ptr<NavMdp> nav_model = boost::dynamic_pointer_cast<NavMdp>(model); 
    nav_model->setTerminalState(terminal); 
    std::cout << "best action at state " << s << ", terminal " << terminal 
        << " is " << vi.getBestAction(s) << std::endl; 

    return 0; 
}
