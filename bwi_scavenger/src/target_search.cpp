#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <string>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// path
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

geometry_msgs::PoseWithCovarianceStamped curr_pos; 

// callback function that saves robot's current position
void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    
    ROS_INFO("I heard something about my current position"); 
    curr_pos = *msg;
    
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

        x_diff = path->poses[i].pose.position.x - path->poses[i-1].pose.position.x; 
        y_diff = path->poses[i].pose.position.y - path->poses[i-1].pose.position.y; 

        len += pow( (x_diff * x_diff) + (y_diff * y_diff), 0.5); 
        
    }

    return len; 
    
}

// this function updates the belief distribution based on bayesian equation
void update_belief(std::vector<float> *belief, bool detected, int next_goal_index) {
    
    float true_positive_rate, true_negative_rate; 

    ros::param::param <float> ("~true_positive_rate", true_positive_rate, 0.8); 
    ros::param::param <float> ("~true_negative_rate", true_negative_rate, 0.8); 
    
    std::vector <float> tmp_belief (*belief); 

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

    for (int i = 0; i < tmp_belief.size(); i++)
        (*belief)[i]  = tmp_belief[i] / normalizer; 
        
}

// this function calls the robot platform and visual sensors to sense a specific
// scene. It return a boolean value identifies if the target is detected. 
bool observe(geometry_msgs::PoseWithCovarianceStamped *curr_pos, 
    ros::Publisher *pub) {

    const float PI = atan(1) * 4; 

    double roll, pitch, yaw;

    geometry_msgs::PoseStamped msg_goal; 
   
    msg_goal.header.frame_id = "level_mux/map";
    msg_goal.pose.position.x = curr_pos->pose.pose.position.x; 
    msg_goal.pose.position.y = curr_pos->pose.pose.position.y; 

    // convert from QuaternionMsg to TFQuaternion
    tf::Quaternion q; 
    tf::quaternionMsgToTF(curr_pos->pose.pose.orientation, q); 

    // compute yaw
    tf::Matrix3x3 mat(q);
    mat.getRPY(roll, pitch, yaw);

    // assemble a goal to let robot rotate
    msg_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + PI/2.0);

    ROS_INFO("Look to the left...");
    pub->publish(msg_goal); 

    msg_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw - PI);

    ROS_INFO("Look to the right...");
    pub->publish(msg_goal); 

    // here we assume the robot can never detects the target, so it's always
    // returning false; 
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
    if (!ros::param::get("~tolerance", tolerance)) 
        ROS_ERROR("tolerance value not set"); 

    float analyzing_cost;
    ros::param::param <float> ("~analyzing_scene_cost", analyzing_cost, 5.0); 

    bool found = false, detected = false;

    ros::Rate loop_rate(10); 

    // break if ros program gets killed or target is found
    while (!found && ros::ok()) {

        // to get the current position using the callback function
        ros::spinOnce(); 
        ROS_INFO("Compute which scene to analyze next"); 

        for (unsigned i = 0; i < positions.size(); i++) {

            // to determine the starting point
            srv.request.start.header.frame_id = "level_mux/map"; 
            srv.request.start.pose = curr_pos.pose.pose; 

            // to determine the goal point
            srv.request.goal.header.frame_id = "level_mux/map"; 
            positions[i][0] >> srv.request.start.pose.position.x; 
            positions[i][1] >> srv.request.start.pose.position.y; 

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

            fitness[i] = belief[i] / distances[i]; 
            
            // finds the largest fitness value and save its index to
            // next_goal_index
            if (fitness[i] >= tmp_max) {
                next_goal_index = i; 
                tmp_max = fitness[i];     
            }
        }

        // assumble a goal for analyzing the next scene of interest
        msg_goal.header.frame_id = "level_mux/map";
        positions[next_goal_index][0] >> msg_goal.pose.position.x; 
        positions[next_goal_index][1] >> msg_goal.pose.position.y; 

        float tmp_z;
        positions[next_goal_index][2] >> tmp_z; 
        msg_goal.pose.orientation.z = tmp_z; 
        msg_goal.pose.orientation.w = pow(1 - (tmp_z * tmp_z), 0.5); 
        
        // we have assumbled a goal, we now publish it to the proper topic
        pub_move_robot.publish(msg_goal); 

        ROS_INFO("Moving to the next scene for visual analyzation"); 

        while (ros::ok()) {

            ros::spinOnce(); 
            float tmp_x = msg_goal.pose.position.x - curr_pos.pose.pose.position.x;
            float tmp_y = msg_goal.pose.position.y - curr_pos.pose.pose.position.y;

            if (pow( tmp_x*tmp_x + tmp_y*tmp_y, 0.5) < tolerance) {
                break;
            }

            loop_rate.sleep();
        }

        ROS_INFO("Arrived"); 

        detected = observe( &curr_pos, &pub_move_robot); 

        // update belief based on observation (true or false)
        update_belief( & belief, detected, next_goal_index); 
        
        std::stringstream ss;
        for (int i=0; i < belief.size(); i++) 
            ss << belief[i] << ", "; 

        std::string str(ss.str());
        ROS_INFO("%s", str.c_str()); 

    }
 
    return 0;        
}


