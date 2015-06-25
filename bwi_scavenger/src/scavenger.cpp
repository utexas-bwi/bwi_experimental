

#include "ScavengerTask.h"
#include "TaskManager.h"

#include "TaskColorShirt.h"
#include "TaskWhiteBoard.h"
#include "TaskFetchObject.h"

#define TIMEOUT (600) // return failure if not finished in 10 minutes

enum TaskStatus{ ONGOING, FINISHED, TODO }; 

int main(int argc, char **argv) {

    ros::NodeHandle *nh = new ros::NodeHandle();

    TaskManager task_manager = new TaskManager(); 

    task_manager->addTask(new ScavTask(new ScavTaskColorShirt(nh, ORANGE), TODO)); 
    task_manager->addTask(new ScavTask(new ScavTaskWhiteBoard(nh), TODO)); 
    task_manager->addTask(new ScavTask(new ScavTaskFetchObject(nh, "Coffee", "l3_414b", "l3_420"), TODO)); 

    while (task_manager->allFinished() == false) {
        task_manager->executeNextTask(); 
        task_manager->updateStatusGui(); 
    }

    return 0; 
}
