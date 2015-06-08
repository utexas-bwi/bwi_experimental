

#include "bwi_nav_reasoning/DomainParser.h"
#include <fstream.h>
#include <boost/lexical_cast.hpp>


DomainParser::DomainParser(const std::string static_obs, 
    const std::string dynamic_obs, const std::string sunny_cells, 
    const std::string plog_facts) : 
    file_static_obstacle{static_obs}, file_dynamic_obstacle{dynamic_obs}, 
    file_sunny_cells{sunny_cells}, file_plog_facts{plog_facts} {

    std::vector<std::vector<int>> vec_static_obstacles; 
    std::vector<std::vector<int>> vec_dynamic_obstacles; 
    std::vector<std::vector<int>> vec_sunny_cells; 

    parseFile(file_static_obstacle, vec_static_obstacles); 
    parseFile(file_dynamic_obstacle, vec_dynamic_obstacles); 
    parseFile(file_sunny_cells, vec_sunny_cells); 

    row_num = vec_static_obstacles.size();
    col_num = vec_static_obstacles[0].size(); 

    int cnt = 0; 
    vector<int> key(2, 0); 
    for (int i=0; i<row_num; i++) {
        for (int j=0; j<col_num; j++) {
            // when the cell is open
            if (vec_static_obstacles[i][j] == 1) {
                State s;
                s.row = i;
                s.col = j;
                s.index = cnt++; 
                s.under_sunlight = (vec_sunny_cells == 1); 
                s.has_human = (vec_dynamic_obstacles == 1); 
                key[0] = i;
                key[1] = j;
                states_map[key] = s; 
            }
        }
    }
}

void DomainParser::parseFile(const std::string filename,
    std::vector<std::vector<int>>& ret) {
 
    ret.clear();    
    ifstream input_file(filename); 

    if (input_file) {
        std::string str;
        vector<int> vec;
        while ( input_file >> str) {
            if (str.find("\n" != str.end())) {
                ret.push_back(vec);
                vec.clear();
            } else {
                vec.push_back(boost::lexical_cast<int>(str)); 
            }
        }
    }
}


