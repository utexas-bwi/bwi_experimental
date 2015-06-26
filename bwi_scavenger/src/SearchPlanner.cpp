

#include <geometry_msgs/Twist.h> 
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "SearchPlanner.h"

#define SCENE_ANALYZATION_COST (60)

geometry_msgs::PoseWithCovarianceStamped curr_position; 
bwi_scavenger::VisionTaskGoal *goal; 

ros::NodeHandle * nh; 

actionlib::SimpleActionClient <bwi_scavenger::VisionTaskAction> * ac; 

// callback function that saves robot's current position
void callbackCurrPos(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    curr_position = *msg;
}

SearchPlanner::SearchPlanner(ros::NodeHandle *nh, std::string path_to_yaml, float goal_tolerance) : 
        tolerance(goal_tolerance) {

    YAML::Node yaml_positions = YAML::LoadFile(yaml_file_positions); 

    belief = std::vector<float> (yaml_positions.size(), 1.0/yaml_positions.size()); 

    ros::ServiceClient client_make_plan = i
        nh->serviceClient <nav_msgs::GetPlan> ("move_base/NavfnROS/make_plan"); 

    ros::Publisher pub_simple_goal = 
        nh->advertise<geometry_msgs::PoseStamped>("/move_base_interruptable_simple/goal", 100);

    ros::Subscriber sub_amcl_pose = 
        nh->subscribe("amcl_pose", 100, callbackCurrPos); 
    
    for (int i=0; i<yaml_positions.size(); i++) {
        geometry_msgs::PoseStamped tmp_pose; 
        tmp_pose.header.frame_id = "level_mux/map";
        tmp_pose.pose.position.x = yaml_positions[i][0].as<double>(); 
        tmp_pose.pose.position.y = yaml_positions[i][1].as<double>(); 
        tmp_pose.pose.orientation.z = yaml_positions[i][2].as<double>(); 
        tmp_pose.pose.orientation.w = yaml_positions[i][3].as<double>(); 
        positions.push_back(tmp_pose); 
    }        

    setTargetDetection(false); 
    search(); 

}

geometry_msgs::PoseStamped SearchPlanner::selectNextScene(const std::vector<float> &belief, int &next_goal_index) {

    geometry_msgs::PoseStamped nextScene; 

    nav_msgs::GetPlan srv; 

    srv.request.start.header.frame_id = "level_mux/map"; 
    srv.request.start.pose = curr_position.pose.pose; 
    srv.request.goal.header.frame_id = "level_mux/map"; 

    std::vector<float> distances(belief.size(), 0.0); 
    std::vector<float> fitness(belief.size(), 0.0); 
    
    for (unsigned i=0; i<positions.size(); i++) {

        srv.request.goal.pose.position.x = positions[i][0].as<double>(); 
        srv.request.goal.pose.position.y = positions[i][1].as<double>(); 
    
        client_make_plan.waitForExistence();
        if (false == client_make_plan.call(srv))
            ROS_ERROR("Failed in calling service for making a path"); 
    
        distances[i] = srv.response.plan->poses.size();
    }
    
    next_goal_index = -1; 
    float max_value = -1.0; 

    float resolution;
    ros::param::param <float> ("/move_base/local_costmap/resolution", resolution, 0.05); 
    
    for (unsigned i=0; i<positions.size(); i++) {
        
        fitness[i] = belief[i] / (distances[i]*resolution + SCENE_ANALYZATION_COST); 
        next_goal_index = (fitness[i] >= max_value) ? i : next_goal_index; 
        max_value = (fitness[i] >= max_value) ? fitness[i] : max_value;
    }
    
    nextScene.header.frame_id = "level_mux/map";
    nextScene.pose.position.x = positions[next_goal_index][0].as<double>(); 
    nextScene.pose.position.y = positions[next_goal_index][1].as<double>(); 
    nextScene.pose.orientation.z = positions[next_goal_index][2].as<double>(); 
    nextScene.pose.orientation.w = positions[next_goal_index][3].as<double>(); 
    return nextScene; 
}

void SearchPlanner::moveToNextScene(const geometry_msgs::PoseStamped &msg_goal) {

    bool hasArrived = false; 

    while (ros::ok() and !hasArrived) {

        float x = msg_goal.pose.position.x - curr_position.pose.pose.position.x;
        float y = msg_goal.pose.position.y - curr_position.pose.pose.position.y;
        float dis = pow(x*x + y*y, 0.5); 

        ROS_INFO("distance to next goal: %f", dis); 
        hasArrived = dis < tolerance;

        // sometimes motion plannerg gets aborted for unknown reasons, so here 
        // we periodically re-send the goal
        pub_move_robot.publish(msg_goal); 
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Arrived"); 
}

void SearchPlanner::analyzeScene(float angle, float angular_vel) {

    ros::Publisher pub = nh->advertise <geometry_msgs::Twist> ("/cmd_vel", 100); 
    geometry_msgs::Twist vel; 
    vel.linear.x = 0;
    vel.linear.y = 0;

    for (int i=0; i<170; i++) {
        if (i<10 or i>160)
            vel.angular.z = 0;
        } else if (i<60) {
            ROS_INFO("Look to the left..."); 
            vel.angular.z = angular_vel;
        } else if (i<160)
            ROS_INFO("Look to the right...");
            vel.angular.z = (-1.0) * angular_vel;
        } 
        ros::spinOnce();
        pub.publish(vel); 
        ros::Duration(0.1).sleep();
    }
}

void SearchPlanner::updateBelief(int next_goal_index) {
    
    float true_positive_rate = true_negative_rate = 0.95; 

    std::vector <float> tmp_belief (belief); 

    // if (detected)
    //     tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * true_positive_rate / 
    //         ((tmp_belief[next_goal_index] * true_positive_rate) + 
    //          ((1.0 - tmp_belief[next_goal_index]) * (1.0 - true_negative_rate))); 
    // else
    //     tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * (1.0 - true_positive_rate) / 
    //         ((tmp_belief[next_goal_index] * (1.0 - true_positive_rate)) + 
    //          ((1.0 - tmp_belief[next_goal_index]) * true_negative_rate)); 

    tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * 
                                  (1.0 - true_positive_rate) / 
                                  ( (tmp_belief[next_goal_index] * (1.0 - true_positive_rate)) + 
                                    ((1.0 - tmp_belief[next_goal_index]) * true_negative_rate)
                                  ); 
    float normalizer = 0.0; 

    for (int i = 0; i < tmp_belief.size(); i++)
        normalizer += tmp_belief[i]; 

    for (int i = 0; i < tmp_belief.size(); i++)
        belief[i]  = tmp_belief[i] / normalizer; 
}

void SearchPlanner::search() {

    int next_goal_index; 
    while (ros::ok() and false == getTargetDetection()) {
        moveToNextScene( selectNextScene(belief, next_goal_index) ); 
        analyzeScene(0.25*PI, PI/10.0); 
        updateBelief(next_goal_index); 
    }
}

