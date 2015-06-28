
#include <ros/ros.h>
#include <ros/package.h>

#include "ScavTask.h"
#include "TaskManager.h"
#include "SearchPlanner.h"

#include "ScavTaskColorShirt.h"
#include "ScavTaskWhiteBoard.h"
#include "ScavTaskFetchObject.h"

#define TIMEOUT (600) // return failure if not finished in 10 minutes



int main(int argc, char **argv) {

    ros::init(argc, argv, "scavenger");
    ros::NodeHandle *nh = new ros::NodeHandle();

    TaskManager* task_manager = new TaskManager(nh); 

    std::string dir("/home/bwi/shiqi/"); 
    task_manager->addTask(new TaskWithStatus(new ScavTaskColorShirt(nh, dir, ORANGE), TODO)); 
    task_manager->addTask(new TaskWithStatus(new ScavTaskWhiteBoard(nh, dir), TODO)); 
    task_manager->addTask(new TaskWithStatus(new ScavTaskFetchObject(nh, dir, "Coffee", "l3_414b", "l3_420"), TODO)); 

    while (task_manager->allFinished() == false) {
        task_manager->executeNextTask(600); 
        task_manager->updateStatusGui(); 
    }

    return 0; 
}
