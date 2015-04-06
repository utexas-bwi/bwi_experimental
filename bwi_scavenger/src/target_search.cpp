#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <vector>
#include <fstream>

geometry_msgs::PoseStamped current; 

// callback function that saves robot's current position
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
 
    current = msg->data;    
    
}

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
    std::vector <int> distances(positions.size(), 0); 

    // service for computing a path from current position to a goal
    ros::ServiceClient client = nh.serviceClient <nav_msgs::GetPlan> 
        ("move_base/NavfnROS/make_plan"); 
    nav_msgs::GetPlan srv; 

    // subscribe to the topic that reports the current position
    ros::Subscriber sub = nh.subscribe("amcl_pose", 100, callback); 

    // get the parameter of tolerance to goal

    bool found = false;

    while (!found) {

        // to get the current position

        for (unsigned i = 0; i < positions.size(); i++) {
        
            // call service to compute a path to a possible positon
            // compute and save the distance of that path

            std::vector<float> vec; 
            positions[i] >> vec; 
            std::cout << vec[0] << ", " << vec[1] << ", " << vec[2] << std::endl;
            
        }

        // weight the probabilities by the distances, select the best one

        // send goal to the symbolic planner, and record observations

        // update belief based on observation (true or false)

        spinOnce(); 
    }
 
    return 0;        
}


