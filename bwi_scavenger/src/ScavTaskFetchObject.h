

#ifndef SCAVTASKFETCHOBJECT_H
#define SCAVTASKFETCHOBJECT_H

#include <string>

#include "ScavTask.h"

class ScavTaskFetchObject : public ScavTask {
public:
    ScavTaskFetchObject();
    ScavTaskFetchObject(ros::NodeHandle *node_handle, std::string object, std::string room_from, std::string room_to) : 
        nh(node_handle), object_name(object), room_name_from(room_from), room_name_to(room_to) {}

    void executeTask(int timeout, TaskResult &result, std::string &record); 

    std::string object_name, room_name_from, room_name_to; 

}


#endif
