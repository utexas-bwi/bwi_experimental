#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <vector>
#include <fstream>

geometry_msgs::PoseStamped curr_pos; 

// callback function that saves robot's current position
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
 
    curr_pos = msg->data;    
    
}


// compute the length of a path, nav_msgs::Path
// we used pixel as the distance unit, as we do not care the absolute values
// returns a float value
float compute_path_length(nav_msgs::Path * path) {
 
    float len = 0.0; 

    if (!path) {
        ROS_ERROR("Failed to compute path length as the path is empty"); 
        return 0.0; 
    }

    if (sizeof(path->poses) / sizeof(path->poses[0]) <= 1) {
        ROS_WARN("No need to compute path length as you have arrived"); 
        return 0.0; 
    }

    for (int i = 1; i < sizeof(path->poses) / sizeof(path->poses[0]); i++) {

        float x_diff, y_diff; 

        x_diff = path->poses[i].position.x - path->poses[i-1].position.x; 
        y_diff = path->poses[i].position.y - path->poses[i-1].position.y; 

        len += pow( (x_diff * x_diff) + (y_diff * y_diff), 0.5); 
        
    }

    return len; 
    
}

// this function updates the belief distribution based on bayesian equation
void update_belief(vector<float> *belief, bool detected, int next_goal_index) {
    
    float true_positive_rate, true_negative_rate; 

    ros::param::param <float> ("~true_positive_rate", true_positive_rate, 0.8); 
    ros::param::param <float> ("~true_negative_rate", true_negative_rate, 0.8); 
    
    vector <float> tmp_belief (*belief); 

    if (detected)
        tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * true_positive_rate / 
            ((tmp_belief[next_goal_index] * true_positive_rate) + 
             ((1.0 - tmp_belief[next_goal_index]) * (1.0 - true_negative_rate))); 
    else
        tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * (1.0 - true_positive_rate) / 
            ((tmp_belief[next_goal_index] * (1.0 - true_positive_rate)) + 
             ((1.0 - tmp_belief[next_goal_index]) * true_negative_rate)); 

    float normalizer = 0.0; 

    for (int i = 0; i < tmp_belief.size(); i++)
        normalizer += tmp_belief[i]; 

    for (int i = 0; i < tmp_belief.size(); i++) {
        (*belief)[i]  = tmp_belief[i] / normalizer; 
        
}

// this function calls the robot platform and visual sensors to sense a specific
// scene. It return a boolean value identifies if the target is detected. 
bool observe() {

    return false; 
    
}

// this is the entrance of the program
int main(int argc, char **argv) {

    ros::init(argc, argv, "target_search");

    ros::NodeHandle nh; 

    std::string file_positions; 

    if (ros::param::get("~positions", file_positions))
        ROS_INFO("\nFile: %s", file_positions.c_str()); 
    else
        ROS_ERROR("Cannot read file"); 

    std::ifstream fin(file_positions.c_str());
    YAML::Parser parser(fin);

    YAML::Node positions;

    parser.GetNextDocument(positions); 

    // belief distribution over all possible target positions
    std::vector <float> belief(positions.size(), 1.0/positions.size()); 

    // a vector that saves the distances to all possible positions
    std::vector <float> distances(positions.size(), 0.0); 

    // fitness function: maximization to decide where to see
    std::vector <float> fitness(positions.size(), 0.0); 

    // service for computing a path from current position to a goal
    ros::ServiceClient client_compute_path = 
        nh.serviceClient <nav_msgs::GetPlan> ("move_base/NavfnROS/make_plan"); 
    nav_msgs::GetPlan srv; 

    // service for driving robot platform
    ros::Publisher pub_move_robot = nh.advertise <geometry_msgs::PoseStamped> 
        ("/move_base_interruptable_simple/goal", 100); 
    geometry_msgs::PoseStamped msg_goal; 

    // subscribe to the topic that reports the current position
    ros::Subscriber sub = nh.subscribe("amcl_pose", 100, callback); 

    // get the parameter of tolerance to goal
    float tolerance; 
    if (!ros::param::get("tolerance", tolerance)) 
        ROS_ERROR("tolerance value not set"); 

    bool found = false, detected = false;

    ros::Rate loop_rate(10); 

    // break if ros program gets killed or target is found
    while (!found && ros::ok()) {

        // to get the current position using the callback function
        ros::spinOnce(); 

        for (unsigned i = 0; i < positions.size(); i++) {

            // to determine the starting point
            srv.request.start.frame_id = "level_mux/map"; 
            srv.request.start.pose = curr_pos.pose; 

            // to determine the goal point
            srv.request.goal.frame_id = "level_mux/map"; 
            srv.request.start.pose.position.x = positions[i][0]; 
            srv.request.start.pose.position.y = positions[i][1]; 

            // call service to compute a path to a possible positon
            if (!client_compute_path.call(srv))
                ROS_ERROR("Calling service for path computing failed"); 

            nav_msgs::Path path = srv.response.plan; 

            // compute and save the distance of that path
            distances[i] = compute_path_length( & path); 

        }

        // fitness function, weighted belief probabilities
        int next_goal_index; 
        float tmp_max = -1.0; 
        for (unsigned i = 0; i < positions.size(); i++) {

            fitness[i] = belief[i] / distance[i]; 
            
            // finds the largest fitness value and save its index to
            // next_goal_index
            if (fitness[i] >= tmp_max) {
                next_goal_index = i; 
                tmp_max = fitness[i];     
            }
        }

        // assumble a goal for analyzing the next scene of interest
        msg_goal.frame_id = "level_mux/map";
        msg_goal.pose.position->x = positions[next_goal_index][0]; 
        msg_goal.pose.position->y = positions[next_goal_index][1]; 

        float tmp_z = positions[next_goal_index][2]; 
        msg_goal.pose.orientation->z = tmp_z; 
        msg_goal.pose.orientation->w = pow(1 - (tmp_z * tmp_z), 0.5); 
        
        // we have assumbled a goal, we now publish it to the proper topic
        pub_move_robot.publish(msg_goal); 

        detected = observe(); 

        // update belief based on observation (true or false)
        belief = update_belief(belief, detected, next_goal_index); 

    }
 
    return 0;        
}




