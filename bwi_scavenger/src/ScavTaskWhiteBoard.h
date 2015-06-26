#ifndef SCAVTASKWHITEBOARD_H
#define SCAVTASKWHITEBOARD_H

#include <string>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "ScavTask.h"
#include "SearchPlanner.h"

struct Pose { 
    float x; float y; 
    Pose() {}
    Pose(float xx, float yy) : x(xx), y(yy) {}
}; 

class ScavTaskWhiteBoard : public ScavTask {
public:

    ScavTaskWhiteBoard() {}
    ScavTaskWhiteBoard(ros::NodeHandle *node_handle, std::string path_of_dir) :
        nh(node_handle), directory(path_of_dir) {
        task_description = "find a person standing near a whiteboard"; 
    }

    void executeTask(int timeout, TaskResut &result, std::string &record); 
    void visionThread();
    void motionThread(); 

    SearchPlanner *search_planner; 

    bool inRectangle(Pose p, Pose top_left, Pose top_right, Pose bottom_left); 
}; 

#endif
