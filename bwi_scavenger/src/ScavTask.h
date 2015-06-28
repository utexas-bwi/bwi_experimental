#ifndef TASKRESULT_H
#define TASKRESULT_H

#include <ros/ros.h>
#include <string>

enum TaskResult{ TIMEOUT, SUCCEEDED, FAILED}; 

class ScavTask {

public:

    ros::NodeHandle *nh; 

    std::string task_description; 
    std::vector<std::string> task_parameters; 

    virtual void executeTask(int timeout, TaskResult &result, std::string &record) {}

}; 

#endif
