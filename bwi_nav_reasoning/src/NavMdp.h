#ifndef NAVMDP_H
#define NAVMDP_H

#include "bwi_rl/planning/PredictiveModel.h"
#include "DomainParser.h"
#include <string>
#include <vector>

#include "utilities.h"
#include <ros/package.h>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

#define EPSILON (0.00001)

static float action_cost=-1.0, success_reward=10.0, failure_penalty=-10.0; 

struct TransValue {
    std::vector<State> ns; 
    std::vector<float> rewards; 
    std::vector<float> probs; 
};    

struct TransKey {
    State state;
    Action action; 
}; 

typedef std::map<TransKey, TransValue> TransMap; 


std::ostream& operator<<(std::ostream& stream, const TransKey& key) {
    stream << key.state;
    stream << " ";
    stream << key.action; 
    return stream; 
}

bool operator<(const TransKey& l, const TransKey& r) {
    if (l.state.row < r.state.row) return true;
    else if (l.state.row > r.state.row) return false;
    if (l.state.col < r.state.col) return true;
    else if (l.state.col > r.state.col) return false; 

    int lact, ract; 
    if (l.action == UP) lact = 0;
    else if (l.action == DOWN) lact = 1;
    else if (l.action == LEFT) lact = 2;
    else lact = 3;

    if (r.action == UP) ract = 0;
    else if (r.action == DOWN) ract = 1;
    else if (r.action == LEFT) ract = 2;
    else ract = 3;

    return lact < ract;
}

bool operator==(const TransKey& l, const TransKey& r) {
    return (l.state.row == r.state.row and l.state.col == r.state.col and l.action == r.action); 
}


class NavMdp : public PredictiveModel<State, Action> {
public:
    std::string file_static_obstacle;
    std::string file_dynamic_obstacle;
    std::string file_sunny_area;
    std::string file_plog_facts; 

    std::vector<std::vector<int> > static_obstacle;
    std::vector<std::vector<int> > dynamic_obstacle;
    std::vector<std::vector<int> > sunny_area;

    std::string tmp_domain_dir; 
    std::string path_to_plog; 

    TransMap trans_map; 

    int terminal_row, terminal_col; 

    DomainParser dparser; 

    NavMdp(std::string static_obs, std::string dynamic_obs, std::string sunny,
        std::string facts, int, int); 

    bool isTerminalState(const State &s) const; 

    void getActionsAtState(const State &s, std::vector<Action>& actions); 

    void getStateVector(std::vector<State>& states); 

    void getTransitionDynamics(const State &s, const Action &a, 
        std::vector<State> &next_state, 
        std::vector<float> &rewards, 
        std::vector<float> &probabilities); 

    void getTransitionDynamicsSlow(const State &s, const Action &a, 
        std::vector<State> &next_state, 
        std::vector<float> &rewards, 
        std::vector<float> &probabilities); 

    std::string generateDescription(unsigned int indentation=0); 

    float getProbability(const State &s, const Action &a, const State &ns); 
}; 

#endif

#include "NavMdp.cpp"
