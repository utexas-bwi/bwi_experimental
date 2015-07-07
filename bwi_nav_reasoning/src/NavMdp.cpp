
#ifndef NAVMDP_CPP
#define NAVMDP_CPP

NavMdp::NavMdp (std::string static_obs, std::string dynamic_obs, 
        std::string sunny, std::string facts, int term_row, int term_col) {

    terminal_row = term_row;
    terminal_col = term_col; 
    
    std::string path; 
    
    std::string zoidberg_plog("/home/shiqi/software/p-log/plog/install/plog");
    std::string sony_laptop_plog("/home/szhang/software/p-log/plog/src/plog");
    std::string segbot_plog("/home/bwi/Downloads/plog/install/plog"); 

    if (boost::filesystem::exists(zoidberg_plog))
        path_to_plog = "cd /tmp && " + zoidberg_plog + " -t "; 
    else if (boost::filesystem::exists(sony_laptop_plog))
        path_to_plog = "cd /tmp && " + sony_laptop_plog + " -t "; 
    else if (boost::filesystem::exists(segbot_plog))
        path_to_plog = "cd /tmp && " + segbot_plog + " -t "; 
    else
        std::cout << "cannot find plog installed" << std::endl; 

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
    int cnt = 0; 
    for (int i=0; i<all_states.size(); i++) {

        if (i*1.0/all_states.size() >= cnt*1.0/100.0) {
            while (i*1.0/all_states.size() > (++cnt)*1.0/100.0) {}
            std::cout << "\rfinished: " << cnt << "\%" << std::endl;
        }

        std::cout << all_states[i] << std::endl; 

        for (int j=0; j<all_actions.size(); j++) {
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

            std::cout << "\t" << all_actions[j] << std::endl;
            for (int ii = 0; ii < tmp_next_states.size(); ii++) {
                std::cout << "\t\tns: " << v.ns[ii] << " reward: " << v.rewards[ii] << " prob: " << v.probs[ii] << std::endl; 
            }
            
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
    if (trans_map.find(k) == trans_map.end()) {
        std::cout << "cannot find key in trans_map" << k << std::endl; 
    } 

    v = trans_map[k]; 
    ns = v.ns;
    rewards = v.rewards;
    probs = v.probs; 

    float sum = 0; 
    for (int i=0; i<ns.size(); i++)
        sum += probs[i]; 

    if (sum > 1.0 + EPSILON or sum < 1.0 - EPSILON)
        std::cout << "Error: probabilities do not sum to 1" << std::endl;

    /* for debugging */
    // std::cout << "state: " << s << " action: " << a << std::endl; 
    // for (int i=0; i<ns.size(); i++)
    //     std::cout << "\tns: " << ns[i] << " reward: " << rewards[i] << " prob: " << probs[i] << std::endl;
}

void NavMdp::getTransitionDynamicsSlow(const State &s, const Action &a, 
    std::vector<State> &ns, std::vector<float> &reward, 
    std::vector<float> &probs) {

    ns.clear();
    // if terminal state, identify the end of this episode
    if (s.row == -1 and s.col == -1) {
        ns.push_back(s);
        reward = std::vector<float>(1, 0.0);
        probs = std::vector<float>(1, 1.0); 
        return; 
    }

    int r = s.row, c = s.col; 

    reward.clear(); 
    probs.clear(); 

    // always has a probability of not moving at all, so state does not change
    ns.push_back(s); 
    reward.push_back(-1.0); 
    probs.push_back(getProbability(s, a, s)); 

    std::vector<int> row_col = std::vector<int>(2, 0); 
    std::map<std::vector<int>, State>::iterator it; 

    // moving down
    row_col[0] = r-1; 
    row_col[1] = c; 
    it = dparser.states_map.find(row_col); 
    if (it != dparser.states_map.end()) {
        ns.push_back(it->second); 
        reward.push_back(isTerminalState(it->second) ? success_reward : action_cost);
        probs.push_back(getProbability(s, a, it->second)); 
    }

    // moving up
    row_col[0] = r+1;
    it = dparser.states_map.find(row_col); 
    if (it != dparser.states_map.end()) {
        ns.push_back(it->second); 
        reward.push_back(isTerminalState(it->second) ? success_reward : action_cost);
        probs.push_back(getProbability(s, a, it->second)); 
    }

    // left
    row_col[0] = r; 
    row_col[1] = c-1; 
    it = dparser.states_map.find(row_col); 
    if (it != dparser.states_map.end()) {
        ns.push_back(it->second);
        reward.push_back(isTerminalState(it->second) ? success_reward : action_cost);
        probs.push_back(getProbability(s, a, it->second)); 
    }

    // right
    row_col[1] = c+1; 
    it = dparser.states_map.find(row_col); 
    if (it != dparser.states_map.end()) {
        ns.push_back(it->second); 
        reward.push_back(isTerminalState(it->second) ? success_reward : action_cost);
        probs.push_back(getProbability(s, a, it->second)); 
    }

    // then add the terminal state into the "next state" list
    row_col[0] = -1; 
    row_col[1] = -1; 
    it = dparser.states_map.find(row_col); 
    if (it != dparser.states_map.end()) {
        ns.push_back(it->second); 
        reward.push_back((s.row == -1 and s.col == -1) ? 0 : failure_penalty);
        probs.push_back(getProbability(s, a, it->second)); 
    }
}

bool NavMdp::isTerminalState(const State &s) const {
    return (s.row == terminal_row) and (s.col == terminal_col); 
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

    int prob_start, prob_end; 

    if (output.find("probability: ") != std::string::npos) {
        prob_start = output.find("probability: ") + 13; 
        prob_end = output.find(" ", prob_start); 
    } else {
        std::cout << "Error in computing probability" << std::endl; 
    }

    std::string str_prob = output.substr(prob_start, prob_end - prob_start); 

    return boost::lexical_cast<float> (str_prob); 
}

std::string NavMdp::generateDescription(unsigned int indentation) {
    return std::string(""); 
}

#endif
