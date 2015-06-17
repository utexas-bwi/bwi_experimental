
#include "NavMdp.h"
#include "getStdoutFromCommand.h"
#include <ros/package.h>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <ifstream>

NavMdp::NavMdp () {}

NavMdp::NavMdp (std::string static_obs, std::string dynamic_obs, 
        std::string sunny, std::string facts) {

    std::string path; 
    path_to_plog = "/home/szhang/tools/plog/plog/install/bin/plog -t "; 

    // if create an empty folder under /tmp/... for temporary files
    std::string tmp_domain_dir = "/tmp/rl_domain/"; 
    if (boost::filesystem::is_directory(tmp_domain_dir)) {
        boost::filesystem::remove_all(tmp_domain_dir); 
    else
        boost::filesystem::create_directory(tmp_domain_dir); 

    // read maps and generate facts to temporary folder
    path = ros::package::getPath("bwi_nav_reasoning") + "/maps/"; 
    dparser = DomainParser(path + static_obs, path + dynamic_obs, path + sunny, 
        tmp_domain_dir + "facts.plog");

    // copy plog rules to temporary folder
    path = ros::package::getPath("bwi_nav_reasoning") + "/domain/"; 
    boost::filesystem::copy_file(path + "rules.plog", tmp_domain_dir); 
}

void NavMdp::getActionAtState(const State &s, std::vector<Action> &actions) {
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

bool NavMdp::isTerminalState(const State &s) {
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

    std::ofstream output_file(tmp_domain_dir + "query.txt"); 
    output_file.write(query); 
    
    std::string cmd; 
    // generate a plog file that includes: query, facts and rules
    cmd += "rm " + tmp_domain_dir + "/all.plog; "; 
    cmd += "cat " + tmp_domain_dir + "/* > " + tmp_domain_dir + "/all.plog; ";
    getStdoutFromCommand(cmd); 

    // call plog solver to compute the probability
    cmd = path_to_plog + tmp_domain_dir + "/all.plog"; 
    std::string output = getStdoutFromCommand(cmd); 
    int prob_start, prob_end; 
    prob_start = output.find("probability: ") + 12; 
    prob_end = output.find(" ", prob_start); 

    return boost::lexical_cast<float> (output(prob_start, prob_end - prob_start)); 
}
