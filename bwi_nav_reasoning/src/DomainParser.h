
#ifndef DOMAINPARSER_HPP
#define DOMAINPARSER_HPP

#include <string>
#include <vector>
#include <map>
#include "StateAction.h"

class DomainParser {

public:

    DomainParser() {}

    DomainParser(const std::string static_obs, const std::string dynamic_obs, 
        const std::string sunny_cells, const std::string plog_facts); 

    std::string file_static_obstacle;
    std::string file_dynamic_obstacle;
    std::string file_sunny_cells; 
    std::string file_plog_facts; 

    std::vector<std::vector<int> > vec_static_obstacles;
    std::vector<std::vector<int> > vec_dynamic_obstacles;
    std::vector<std::vector<int> > vec_sunny_cells;


    int col_num; 
    int row_num; 

    std::map<std::vector<int>, State> states_map; 

    void parseFile(const std::string file, std::vector<std::vector<int> >& vec); 
    void writeToFile(const std::string file); 
}; 

#endif
