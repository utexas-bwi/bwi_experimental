#include "TaskCondition.h"

#include <iostream>
#include <fstream>
#include <ros/package.h>

using namespace std;

double TaskCondition::rew;

TaskCondition::TaskCondition(Client& client) : client(client) {}

bool TaskCondition::evaluateAtomicExternalCondition (const std::string& atom ) {
    string fileName = ros::package::getPath("bwi_tasks_pnp")+"/tasks/"+ atom + ".txt";
    ifstream goalFile(fileName.c_str());

    if ((goalFile) && (goalFile.is_open())) {
        remove(fileName.c_str());
        ROS_INFO("true");
        return true;
    }
    else return false;
}

double TaskCondition::reward() {
    double r = rew;
    rew = 0.;
    //ROS_INFO("reward %F", r);
    return r;
}

void TaskCondition::setReward(double reward) {
    rew = reward;
}
