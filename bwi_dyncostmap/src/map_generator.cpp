#include "map_generator.h"

typedef int32_t* map;

map global_costmap;
map sum_map;
map update_heat_map;
map entropy_map;

void copy_int8_to_int32(const map int8map, const map int32map) {
  for (size_t i = 0; i < sizeof(int8map) / sizeof(int8_t); i++) {
    int32map[i] = int8map[i];
  }
}

void global_costmap_handler(const nav_msgs::OccupancyGrid& msg) {
  ROS_INFO("Received global costmap");
  // copy recv costmap into here global_costmap = 
  size_t height = msg.info.height;
  size_t width  = msg.info.width;
  global_costmap  = new int32_t[height*width];
  sum_map         = new int32_t[height*width];
  update_heat_map = new int32_t[height*width];
  entropy_map     = new int32_t[height*width];
  //TODO : continue here
}

void costmap_update_handler(const map_msgs::OccupancyGridUpdate& update) {
  ROS_INFO("Received update");
}

//void apply_patch(int8_t[]& patch, 

int main(int argc, char **argv){
  ros::init(argc, argv, "map_generator");
  ros::NodeHandle n;

  ros::Subscriber global_sub = n.subscribe("/move_base/global_costmap/costmap", 5, global_costmap_handler);

  ros::Subscriber update_sub = n.subscribe("/move_base/global_costmap/costmap_updates", 1000, costmap_update_handler);

  ROS_INFO("bwi_dyncostmap: Map generator running");
  ros::spin();

  return 0;
}
