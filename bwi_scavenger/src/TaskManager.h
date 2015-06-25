
#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <vector>
#include "ScavengerTask.h"

struct TaskWithStatus {
    ScavTask *task; 
    ScavStatus status; 

    TaskWithStatus() : task(NULL), status(TODO) {}

    TaskWithStatus(ScavTask *scav_task, ScavStatus scav_status) : 
        task(scav_task), status(scav_status) {}
}; 

class TaskManager {

public:

    std::vector<TaskWithStatus* > tasks;

    void addTask(TaskWithStatus *task); 

    void executeNextTask(); 
    void updateStatusGui(); 
    bool allFinished(); 

}


#endif
