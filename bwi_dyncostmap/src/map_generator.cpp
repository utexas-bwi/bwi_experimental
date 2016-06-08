#include "map_generator.h"


void global_costmap_handler(const nav_msgs::OccupancyGrid& costmap) {
  ROS_INFO("Received global costmap");
}

void costmap_update_handler(const map_msgs::OccupancyGridUpdate& update) {
  ROS_INFO("Received update");
}

int main(int argc, char **argv){
  ros::init(argc, argv, "map_generator");
  ros::NodeHandle n;

  ros::Subscriber global_sub = n.subscribe("/move_base/global_costmap/costmap", 5, global_costmap_handler);

  ros::Subscriber update_sub = n.subscribe("/move_base/global_costmap/costmap_updates", 1000, costmap_update_handler);

  ROS_INFO("bwi_dyncostmap: Map generator running");
  ros::spin();

  return 0;
}
