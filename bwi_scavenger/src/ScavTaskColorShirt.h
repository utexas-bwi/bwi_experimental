
#ifndef SCAVTASKCOLORSHIRT_H
#define SCAVTASKCOLORSHIRT_H

#include <ros/ros.h>
#include <string>

#include "ScavTask.h"
#include "SearchPlanner.h"

enum Color{ RED, GREEN, BLUE, YELLOW, ORANGE }; 

class ScavTaskColorShirt : public ScavTask {

public:
 
    ScavTaskColorShirt(); 

    ScavTaskColorShirt(ros::NodeHandle *node_handle, Color shirt_color) : nh(node_handle), color(shirt_color) {}


    void executeTask(int timeout, TaskResult &result, std::string &record); 

    SearchPlanner *search_planner; 

    Color color; 
    std::string path; 

    // to compute the distance between a Rgb struct and a PCL RGB
    float getColorDistance(const pcl::PointXYZRGB *c1, const Rgb *c2) 
    {
        return pow(pow(c1->r- c2->r, 2.0) + pow(c1->g - c2->g, 2.0) + pow(c1->b - c2->b, 2.0), 0.5);
    }

}

#endif
