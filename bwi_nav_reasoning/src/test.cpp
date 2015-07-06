#include <boost/foreach.hpp>
#include <iostream>
#include <bwi_rl/planning/PredictiveModel.h>
#include <bwi_rl/planning/ValueIteration.h>
#include <bwi_rl/planning/VITabularEstimator.h>

#include "NavMdp.h"
#include "Driver.h"

int main(int argc, char **argv) {

    State term, curr, next; 
    term.row = 0;
    term.col = 3;

    std::cout << "creating nav model..." << std::endl; 
    boost::shared_ptr<NavMdp> model(
        new NavMdp("gdc_atrium/static_map_small.txt", 
                   "gdc_atrium/dynamic_map_small.txt", 
                   "gdc_atrium/sunny_map_small.txt", 
                   "tmp/rl_domain/facts.plog", term.row, term.col)); 

    std::cout << "creating vi estimator..." << std::endl; 
    boost::shared_ptr<VIEstimator<State, Action> > estimator(new VITabularEstimator<State, Action>); 

    ValueIteration<State, Action> vi(model, estimator);

    std::cout << "Computing policy..." << std::endl;
    vi.computePolicy(); 

    Driver *driver = new Driver("gdc_atrium/coordinates_real.yaml"); 

    while (false == (curr == term)) {
        driver->updateCurrentState(curr); 
        Action action = vi.getBestAction(curr); 
        std::cout << "best action at state " << curr << ", terminal " << term 
                  << " is " << action << std::endl; 
 
        next = curr;        
        if (action == UP) next.row += 1;
        else if (action == DOWN) next.row -= 1;
        else if (action == LEFT) next.col -= 1;
        else if (action == RIGHT) next.col += 1; 

        driver->moveToGoalState(next); 
    }
    return 0; 
}


