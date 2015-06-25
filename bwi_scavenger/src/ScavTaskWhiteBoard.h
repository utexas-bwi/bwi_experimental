

#ifndef SCAVTASKWHITEBOARD_H
#define SCAVTASKWHITEBOARD_H

#include <string>

#include "ScavTask.h"
#include "SearchPlanner.h"

class ScavTaskWhiteBoard : public ScavTask {

public:

    ScavTaskWhiteBoard();

    void executeTask(int timeout, TaskResut &result, std::string &record); 

    SearchPlanner *search_planner; 

}


#endif
