#include "bwi_nav_reasoning/DomainParser.h"
#include <fstream>
#include <boost/lexical_cast.hpp>

DomainParser::DomainParser(const std::string static_obs, 
    const std::string dynamic_obs, const std::string sunny_cells, 
    const std::string plog_facts) : 
    file_static_obstacle{static_obs}, file_dynamic_obstacle{dynamic_obs}, 
    file_sunny_cells{sunny_cells}, file_plog_facts{plog_facts} {

    parseFile(file_static_obstacle, vec_static_obstacles); 
    parseFile(file_dynamic_obstacle, vec_dynamic_obstacles); 
    parseFile(file_sunny_cells, vec_sunny_cells); 

    row_num = vec_static_obstacles.size();
    col_num = vec_static_obstacles[0].size(); 

    int cnt = 0; 
    std::vector<int> key(2, 0); 
    for (int i=0; i<row_num; i++) {
        for (int j=0; j<col_num; j++) {
            // when the cell is open
            if (vec_static_obstacles[i][j] == 1) {
                State s;
                s.row = i;
                s.col = j;
                s.index = cnt++; 
                s.under_sunlight = (vec_sunny_cells[i][j] == 1); 
                s.has_human = (vec_dynamic_obstacles[i][j] == 1); 
                key[0] = i;
                key[1] = j;
                states_map[key] = s; 
            }
        }
    }

}

void DomainParser::parseFile(const std::string filename,
    std::vector<std::vector<int> >& ret) {
 
    ret.clear();    
    std::ifstream input_file(filename.c_str()); 

    if (input_file) {
        std::string str;
        std::vector<int> vec;
        while ( input_file >> str) {
            if (str.find("\n") != std::string::npos) {
                ret.push_back(vec);
                vec.clear();
            } else {
                vec.push_back(boost::lexical_cast<int>(str)); 
            }
        }
    }
}

void DomainParser::writeToFile(const std::string filename) {

    std::string str(""); 
    int state_num = states_map.size();
    str += "state={0.." + boost::lexical_cast<std::string>(state_num-1) + "}.\n"; 
    str += "terminal(" + boost::lexical_cast<std::string>(state_num-1) + ").\n";

    std::string str_leftof, str_belowof, str_sunny, str_human; 

    for (std::map<std::vector<int>, State>::iterator it=states_map.begin(); 
        it!=states_map.end(); it++) {

        std::vector<int> vec(2, 0); 

        vec[0] = it->first[0]; 
        vec[1] = it->first[1]-1; // to find the one one its left

        if (states_map.find(vec) != states_map.end()) {
            int index_left = states_map.find(vec)->second.index;
            str_leftof += "leftof(" + 
                boost::lexical_cast<std::string>(index_left) + "," + 
                boost::lexical_cast<std::string>(it->second.index) + ").\n"; 
        }

        vec[0] = it->first[0]+1; // to find the one below it
        vec[1] = it->first[1]; 

        if (states_map.find(vec) != states_map.end()) {
            int index_below = states_map.find(vec)->second.index;
            str_belowof += "belowof(" + 
                boost::lexical_cast<std::string>(index_below) + "," +
                boost::lexical_cast<std::string>(it->second.index) + ").\n";
        }

        if (it->second.under_sunlight) {
            str_sunny += "sunny(" + 
                boost::lexical_cast<std::string>(it->second.index) + ").\n"; 
        }

        if (it->second.has_human) {
            str_human += "human(" + 
                boost::lexical_cast<std::string>(it->second.index) + ").\n"; 
        }
    }

    std::ofstream output_file(filename.c_str());
    if (output_file.is_open()) {
        output_file << str_leftof << str_belowof << str_sunny << str_human; 
    }
    output_file.close(); 
}













