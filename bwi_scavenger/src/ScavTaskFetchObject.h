

#ifndef SCAVTASKFETCHOBJECT_H
#define SCAVTASKFETCHOBJECT_H

#include <string>
#include <ros/ros.h> 
#include <boost/thread.hpp>

#include "ScavTask.h"

class ScavTaskFetchObject : public ScavTask {
public:

    ScavTaskFetchObject();
    ~ScavTaskFetchObject() { delete gui_service_client }

    ScavTaskFetchObject(ros::NodeHandle *node_handle, std::string path_of_dir, 
        std::string object, std::string room_from, std::string room_to) : 
        nh(node_handle), directory(path_of_dir, object_name(object), 
        room_name_from(room_from), room_name_to(room_to) {
        task_description = "fetch an object from a place to another"; 
    }

    void executeTask(int timeout, TaskResult &result, std::string &record); 
    void hriThread();
    void motionThread(); 

    std::string object_name, room_name_from, room_name_to; 

    std::string directory; 

    ros::ServiceClient *gui_service_client; 

}; 

#endif
