#include "map_generator.h"

#define MAPSIZE (global_height * global_width)
#define px(X,Y) ((X) + global_width * (Y))

typedef int16_t* map;

/* Global variables */
uint32_t global_height;
uint32_t global_width;
bool global_set = false;

/* Global maps */
map global_costmap;
map sum_map;
map update_heat_map;
map entropy_map;

void copy_int8_vector_to_int16_array(const std::vector<int8_t> int8vector, const map int16map) {
  size_t i = 0;
  for(std::vector<int8_t>::const_iterator it = int8vector.begin(); it != int8vector.end(); ++it) {
      int16map[i++] = *it;
  }
}

void map_to_img(const map _map, const std::string filename) {
  // rows, columns, image type (16 bit, signed, 1-chan), data, params
  cv::Mat img(global_height, global_width, CV_16SC1, _map, cv::Mat::AUTO_STEP);

  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);

  cv::imwrite(filename, img, compression_params);
}

void global_costmap_handler(const nav_msgs::OccupancyGrid& msg) {
  if (global_set) return;
  ROS_INFO("Received global costmap");

  global_height = msg.info.height;
  global_width  = msg.info.width;

  global_costmap  = new int16_t[MAPSIZE];
  sum_map         = new int16_t[MAPSIZE];
  update_heat_map = new int16_t[MAPSIZE];
  entropy_map     = new int16_t[MAPSIZE];

  copy_int8_vector_to_int16_array(msg.data, global_costmap);

  /* debug 
  for (size_t i = px(500,500); i < px(550,500); i++) {
    printf("%u ",global_costmap[i]);
    global_costmap[i] = 255;
  }
  printf("\n");
  */

  map_to_img(global_costmap, "test.png");
  global_set = true;
}

void costmap_update_handler(const map_msgs::OccupancyGridUpdate& update) {
  if (!global_set) {
    ROS_INFO("Update received without global set");
    return;
  } else {
    ROS_INFO("Received update");
  }

  uint32_t u_width  = update.width;
  uint32_t u_height = update.height;
  int32_t  u_x      = update.x;
  int32_t  u_y      = update.y;
  std::vector<int8_t> u_data = update.data;

  // skip map resets
  if (u_width == global_width && u_height == global_height) return;

  // apply the patch
  size_t i = 0;
  for(std::vector<int8_t>::const_iterator it = u_data.begin(); it != u_data.end(); ++it) {
    uint32_t x_index = i/u_width + u_y;
    uint32_t y_index = i%u_width + u_x;

    // update the update heat map
    update_heat_map[px(x_index,y_index)] += 1;

    // update the sum map
    sum_map[px(x_index,y_index)] += 1 ? *it > 50 : 0;

    // TODO: calculate entropy and apply the patch
    // TODO: ensure this actually works and all index calculations are right
    // TODO: find way of displaying matrices as an image
    i++;
  }
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
