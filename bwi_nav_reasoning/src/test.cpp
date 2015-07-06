
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <bwi_rl/planning/PredictiveModel.h>
#include <bwi_rl/planning/ValueIteration.h>
#include <bwi_rl/planning/VITabularEstimator.h>

#include "NavMdp.h"
#include "Driver.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "bwi_nav_reasoning_node"); 
    ros::NodeHandle *nh = new ros::NodeHandle(); 

    State term, curr, next; 
    term.row = 4;
    term.col = 1;

    std::string path_static, path_dynamic, path_sunny, path_coord; 
    std::string param_static_name("path_static"), 
                param_dynamic_name("path_dynamic"), 
                param_sunny_name("path_sunny"), 
                param_coord_name("path_coord"); 

    if (ros::param::has(param_static_name)) ros::param::get(param_static_name, path_static); 
    if (ros::param::has(param_dynamic_name)) ros::param::get(param_dynamic_name, path_dynamic); 
    if (ros::param::has(param_sunny_name)) ros::param::get(param_sunny_name, path_sunny); 
    if (ros::param::has(param_coord_name)) ros::param::get(param_coord_name, path_coord); 
        
    std::cout << "creating nav model..." << std::endl; 

    boost::shared_ptr<NavMdp> model(new NavMdp(path_static, path_dynamic, 
        path_sunny, "tmp/rl_domain/facts.plog", term.row, term.col)); 

    std::cout << "creating vi estimator..." << std::endl; 
    boost::shared_ptr<VIEstimator<State, Action> > estimator(new VITabularEstimator<State, Action>); 

    ValueIteration<State, Action> vi(model, estimator);

    std::cout << "Computing policy..." << std::endl;
    vi.computePolicy(); 

    Driver *driver = new Driver(nh, path_coord); 

    Action action; 
    while (ros::ok()) {

        ros::spinOnce(); 
        ros::Duration(1.0).sleep(); 
        driver->updateCurrentState(curr); 

        if (curr == term) 
            break; 
        else 
            action = vi.getBestAction(curr); 
 
        next = curr;        
        if (action == UP) next.row -= 1;
        else if (action == DOWN) next.row += 1;
        else if (action == LEFT) next.col -= 1;
        else if (action == RIGHT) next.col += 1; 
        else ROS_ERROR("action not recognized"); 

        std::cout << "current: " << curr << " next: " << next 
                  << " action: " << action << std::endl; 

        driver->moveToGoalState(next); 
    }
    ROS_INFO("task done!"); 
    return 0; 
}

