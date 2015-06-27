
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <boost/lexical_cast.hpp>

#include "bwi_msgs/QuestionDialog.h"
#include "TaskManager.h"

TaskManager::TaskManager (ros::NodeHandle *nh) {
    this->nh = nh; 
    gui_client = nh->serviceClient <bwi_msgs::QuestionDialog> ("question_dialog");
}

void TaskManager::addTask(TaskWithStatus task_with_status) {
    tasks.push_back(task_with_status); 
}

void TaskManager::executeNextTask(int timeout) {
    for (int i=0; i<tasks.size(); i++) {
        if (tasks[i].status == ONGOING)
            return; 
        else if (tasks[i].status == TODO)
            tasks[i].task->executeTask(timeout); 
    }
}

void TaskManager::updateStatusGui() {

    std::string message(""); 

    for (int i=0; i<tasks.size(); i++) {
        if (tasks.status == ONGOING) {
            message += "->\t" + tasks.task->task_description; 
        } else if (tasks.status == TODO) {
            message += "\t" + tasks.task->task_description; 
        } else if (tasks.status == FINISHED) {
            message += "done" + tasks.task->task_description; 
        }

        for (int j=0; j<tasks.task->task_parameters.size(); j++) {
            message += " " + tasks.task->task_parameters[j]; 
        }
        message += "\n"; 
    }
    
    srv.request.type = 0;
    srv.request.message = message; 

    gui_service_client->call(srv);
}


bool TaskManager::allFinished() {
    for (int i=0; i<tasks.size(); i++) {
        if (tasks[i].status != FINISHED)
            return false; 
    }
    return true; 
}

