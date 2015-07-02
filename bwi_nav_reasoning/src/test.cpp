#include <boost/foreach.hpp>
#include <iostream>
#include <bwi_rl/planning/PredictiveModel.h>
#include <bwi_rl/planning/ValueIteration.h>
#include <bwi_rl/planning/VITabularEstimator.h>

#include "NavMdp.h"
#include "StateAction.h"
#include "DomainParser.h"

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

    State start, term; 
    term.row = 0;
    term.col = 3;
    start.row = 2;
    start.col = 4; 

    std::cout << "creating nav model..." << std::endl; 
    boost::shared_ptr<NavMdp> model(new NavMdp("gdc_atrium/static_map_small.txt", "gdc_atrium/dynamic_map_small.txt", 
        "gdc_atrium/sunny_map_small.txt", "tmp/rl_domain/facts.plog", term.row, term.col)); 

    std::cout << "creating vi estimator..." << std::endl; 
    boost::shared_ptr<VIEstimator<State, Action> > estimator(new VITabularEstimator<State, Action>); 

    // compute policy
    ValueIteration<State, Action> vi(model, estimator);

    std::cout << "Computing policy..." << std::endl;
    vi.computePolicy(); 


    std::cout << "best action at state " << start << ", terminal " << term << " is " << vi.getBestAction(start) << std::endl; 

    return 0; 
}


