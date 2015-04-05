
#include <ros/ros.h>
#include <vector>

int main(int argc, char **argv) {

    std::vector<int> positions; 

    ros::param::get("positions", positions); 
 
    return 0;        
}
