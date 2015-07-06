#include "DomainParser.h"
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>

DomainParser::DomainParser(const std::string static_obs, 
    const std::string dynamic_obs, const std::string sunny_cells, 
    const std::string plog_facts) {

    file_static_obstacle = static_obs; 
    file_dynamic_obstacle = dynamic_obs; 
    file_sunny_cells = sunny_cells;
    file_plog_facts = plog_facts; 

    vec_static_obstacles = std::vector<std::vector<int> >(0, std::vector<int>(0,0));
    vec_dynamic_obstacles = std::vector<std::vector<int> >(0, std::vector<int>(0,0));
    vec_sunny_cells = std::vector<std::vector<int> >(0, std::vector<int>(0,0));

    std::cout << "parsing model files: " << file_static_obstacle << std::endl;
    parseFile(file_static_obstacle, vec_static_obstacles); 
    std::cout << "parsing model files: " << file_dynamic_obstacle << std::endl;
    parseFile(file_dynamic_obstacle, vec_dynamic_obstacles); 
    std::cout << "parsing model files: " << file_sunny_cells << std::endl;
    parseFile(file_sunny_cells, vec_sunny_cells); 

    std::cout << "finished parsing model files" << std::endl;

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

    State s;
    s.row = s.col = -1;
    s.index = cnt;
    key[0] = -1;
    key[1] = -1;
    states_map[key] = s; 

    std::cout << "finished domain files parsing" << std::endl; 
    
    std::cout << "writing to file: " << file_plog_facts << std::endl;
    writeToFile(file_plog_facts); 
    std::cout << "writing to file finished" << std::endl; 
}

void DomainParser::parseFile(const std::string filename,
    std::vector<std::vector<int> >& ret) {
 
    ret.clear();    
    std::ifstream input_file(filename.c_str()); 
    int num_row, num_col; 

    if (!input_file) {
        std::cout << "could not open file: " << filename << std::endl; 
    }

    input_file >> num_row; 
    input_file >> num_col; 
    for (int i=0; i<num_row; i++) {
        std::vector<int> vec;
        for (int j=0; j<num_col; j++) {
            std::string str;
            input_file >> str; 
            vec.push_back(boost::lexical_cast<int>(str)); 
        }
        ret.push_back(vec);
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
    str += str_leftof + str_belowof + str_sunny + str_human;

    if (output_file.is_open()) {
        output_file << str; 
    }
    output_file.close(); 

}

