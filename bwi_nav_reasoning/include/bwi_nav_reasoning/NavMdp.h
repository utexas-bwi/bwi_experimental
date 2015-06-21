#ifndef NAVMDP_h
#define NAVMDP_H

#include "bwi_rl/planning/PredictiveModel.h"
#include "bwi_nav_reasoning/StateAction.h"
#include "bwi_nav_reasoning/DomainParser.h"
#include <string>
#include <vector>

#include "getStdoutFromCommand.h"
#include <ros/package.h>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

struct TransValue {
    std::vector<State> ns; 
    std::vector<float> rewards; 
    std::vector<float> probs; 
};    

struct TransKey {
    State state;
    Action action; 
}; 

bool operator<(const TransKey& l, const TransKey& r) {
    if (l.state.index < r.state.index) return true;
    if (l.state.index > r.state.index) return false;

    int la = l.action, ra = r.action; 
    if (la < ra) return true;
    if (la > ra) return false;
}

bool operator==(const TransKey& l, const TransKey& r) {
    return (l.state.index == r.state.index && l.action == r.action); 
}

typedef std::map<TransKey, TransValue> TransMap; 

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
        std::string facts); 

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
    void setTerminalState(int row, int col); 
}; 


NavMdp::NavMdp (std::string static_obs, std::string dynamic_obs, 
        std::string sunny, std::string facts) {

    std::string path; 
    path_to_plog = "/home/szhang/software/p-log/plog/src/plog -t "; 

    // if create an empty folder under /tmp/... for temporary files
    tmp_domain_dir = "/tmp/rl_domain/"; 
    if (boost::filesystem::is_directory(tmp_domain_dir)) {
        boost::filesystem::remove_all(tmp_domain_dir); 
    }

    boost::filesystem::create_directory(tmp_domain_dir); 

    // read maps and generate facts to temporary folder
    path = ros::package::getPath("bwi_nav_reasoning") + "/maps/"; 

    std::cout << "creating domain parser..." << std::endl; 
    dparser = DomainParser(path + static_obs, path + dynamic_obs, path + sunny, 
        tmp_domain_dir + "facts.plog");
    std::cout << "creating domain parser finished" << std::endl; 

    // copy plog rules to temporary folder
    path = ros::package::getPath("bwi_nav_reasoning") + "/domain/"; 
    boost::filesystem::copy_file(path + "rules.plog", tmp_domain_dir + "rules.plog"); 

    std::vector<State> all_states, tmp_next_states; 
    std::vector<float> tmp_rewards, tmp_probs; 
    getStateVector(all_states); 
    std::vector<Action> all_actions; 
    getActionsAtState(all_states[0], all_actions);  

    std::cout << "computing transition dynamics..." << std::endl; 
    for (int i=0; i<all_states.size(); i++) {
        for (int j=0; j<all_actions.size(); j++) {
            std::cout << "state index " << i << " action index " << j << std::endl; 
            TransKey k; 
            TransValue v; 

            k.state = all_states[i]; 
            k.action = all_actions[j];

            tmp_next_states.clear();
            tmp_rewards.clear();
            tmp_probs.clear();
            getTransitionDynamicsSlow(k.state, k.action, tmp_next_states, 
                tmp_rewards, tmp_probs); 
            v.ns = tmp_next_states; 
            v.rewards = tmp_rewards;
            v.probs = tmp_probs;
            trans_map[k] = v;
        }
    }

    std::cout << "finished nav model initialization" << std::endl; 
}

void NavMdp::getActionsAtState(const State &s, std::vector<Action> &actions) {
    actions.clear();
    actions.push_back(UP);
    actions.push_back(DOWN);
    actions.push_back(LEFT);
    actions.push_back(RIGHT);
}

void NavMdp::getStateVector(std::vector<State> &states) {

    states.clear(); 
    std::map<std::vector<int>, State>::iterator it; 
    for (it=dparser.states_map.begin(); it!=dparser.states_map.end(); ++it)
        states.push_back(it->second); 
}

void NavMdp::getTransitionDynamics(const State &s, const Action &a, 
    std::vector<State> &ns, std::vector<float> &rewards, 
    std::vector<float> &probs) {
    
    TransKey k;
    TransValue v; 

    k.state = s; 
    k.action = a;
    v = trans_map[k]; 
    ns = v.ns;
    rewards = v.rewards;
    probs = v.probs; 
}
void NavMdp::getTransitionDynamicsSlow(const State &s, const Action &a, 
    std::vector<State> &ns, std::vector<float> &reward, 
    std::vector<float> &probs) {

    int r = s.row, c = s.col; 

    ns.clear();
    reward.clear(); 
    probs.clear(); 

    ns.push_back(s); 
    reward.push_back(-1.0); 
    probs.push_back(getProbability(s, a, s)); 

    std::vector<int> rc = std::vector<int>(2, 0); 
    std::map<std::vector<int>, State>::iterator it; 

    rc[0] = r-1; 
    it = dparser.states_map.find(rc); 
    if (it != dparser.states_map.end()) {
        ns.push_back(it->second); reward.push_back(-1.0); 
        probs.push_back(getProbability(s, a, it->second)); 
    }
    rc[0] = r+1;
    it = dparser.states_map.find(rc); 
    if (it != dparser.states_map.end()) {
        ns.push_back(it->second); reward.push_back(-1.0); 
        probs.push_back(getProbability(s, a, it->second)); 
    }
    rc[0] = r; 
    rc[1] = c-1; 
    it = dparser.states_map.find(rc); 
    if (it != dparser.states_map.end()) {
        ns.push_back(it->second); reward.push_back(-1.0); 
        probs.push_back(getProbability(s, a, it->second)); 
    }
    rc[1] = c+1; 
    it = dparser.states_map.find(rc); 
    if (it != dparser.states_map.end()) {
        ns.push_back(it->second); reward.push_back(-1.0); 
        probs.push_back(getProbability(s, a, it->second)); 
    }
}

bool NavMdp::isTerminalState(const State &s) const {
    return s.row == terminal_row && s.col == terminal_col; 
}

void NavMdp::setTerminalState(int row, int col) {
    terminal_row = row; 
    terminal_col = col; 
}

float NavMdp::getProbability(const State &s, const Action &a, const State &ns) {
    std::string next_s_index, curr_s_index, action_name, query;
    
    curr_s_index = boost::lexical_cast<std::string> (s.index); 
    next_s_index = boost::lexical_cast<std::string> (ns.index);

    if (a == UP) 
        action_name = "up";
    else if (a == DOWN)
        action_name = "down";
    else if (a == LEFT)
        action_name = "left";
    else if (a == RIGHT)
        action_name = "right";

    query = "?{next_s=" + next_s_index + "} | obs(curr_s=" + 
        curr_s_index + "), do(curr_a=" + action_name + "). "; 

    std::ofstream output_file((tmp_domain_dir + "query.txt").c_str()); 
    output_file << query; 
    output_file.close(); 
    
    std::string cmd; 
    boost::filesystem::path file_all(tmp_domain_dir + "/all.plog"); 
    // generate a plog file that includes: query, facts and rules
    
    if (!file_all.empty())
        boost::filesystem::remove(file_all); 

    cmd += "cat " + tmp_domain_dir + "/* > " + tmp_domain_dir + "/all.plog; ";
    getStdoutFromCommand(cmd); 

    // call plog solver to compute the probability
    cmd = path_to_plog + tmp_domain_dir + "/all.plog"; 
    std::string output = getStdoutFromCommand(cmd); 
    std::cout << "output from p-log: \n" << output << std::endl; 

    int prob_start, prob_end; 
    
    if (output.find("probability: ") != string::npos) {
        prob_start = output.find("probability: ") + 13; 
        prob_end = output.find(" ", prob_start); 
    } else {
        std::cout << "Error in computing probability" << std::endl; 
    }

    std::string str_prob = output.substr(prob_start, prob_end - prob_start); 
    std::cout << "str_prob: " << str_prob << std::endl; 

    return boost::lexical_cast<float> (str_prob); 
}

std::string NavMdp::generateDescription(unsigned int indentation) {
    return std::string(""); 
}
#endif
