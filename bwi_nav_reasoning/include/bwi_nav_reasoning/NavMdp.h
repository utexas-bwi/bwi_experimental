#ifndef NAVMDP_h
#define NAVMDP_H

#include "bwi_rl/planning/PredictiveModel.h"
#include "bwi_nav_reasoning/StateAction.h"
#include <string>
#include <vector>


class NavMdp : public PredictiveModel<State, Action> {
public:
    std::string file_static_obstacle;
    std::string file_dynamic_obstacle;
    std::string file_sunny_area;
    std::string file_plog_facts; 

    NavMdp(std::string static_obs, std::string dynamic_obs, std::string sunny,
        std::string facts); 

    bool isTerminalState(const State &s); 

    void getActionsAtState(const State &s, std::vector<Action>& actions); 

    void getStateVector(<std::vector<State>& states>); 

    void getTransitionDynamics(const State &s, const Action &a, 
        std::vector<State> &next_state, 
        std::vector<float> &rewards, 
        std::vector<float> &probabilities); 

    std::string generateDescription(unsigned int indentation=0) {
        return std::string (""); 
    }
}

#endif
