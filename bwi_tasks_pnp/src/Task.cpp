#include "Task.h"

#include "TaskCondition.h"

#include <iterator>

Task::Task(bwi_kr_execution::ExecutePlanGoal &goal, Client &client) : goal(goal), client(client), goalSent(false) {}

void Task::executeStep() {
    if (!goalSent) {
        ROS_INFO("sending goal");
        goalSent = true;
        initial_time = ros::Time::now().toSec();
        client.sendGoal(goal);
    }
}

bool Task::finished() {

    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("Aborted");
        return true;
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_INFO("Preempted");
        return true;
    }
    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Succeeded!");
        double reward = initial_time - ros::Time::now().toSec();
        ROS_INFO("Reward is %f",reward);
        TaskCondition::setReward(reward);
        return true;
    }
    else
        return false;
}
