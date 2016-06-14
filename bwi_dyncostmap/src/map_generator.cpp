#include "map_generator.h"

#define NEWMAP16 new int16_t[MAPSIZE]
#define NEWMAP64 new int64_t[MAPSIZE]
#define MAPSIZE (global_height * global_width)
#define px(X,Y) ((X) + global_width * (Y))

typedef int16_t* map16;
typedef int64_t* map64;

/* Global variables */
uint32_t global_height;
uint32_t global_width;
bool global_set = false;

/* Global maps */
map16 global_costmap;
map64 sum_map;
map16 update_heat_map;
map16 entropy_map;

void copy_int8_vector_to_int16_array(const std::vector<int8_t> int8vector, const map16 int16map) {
  size_t i = 0;
  for(std::vector<int8_t>::const_iterator it = int8vector.begin(); it != int8vector.end(); ++it) {
      int16map[i++] = *it;
  }
}

void map_to_img(const map16 _map, const std::string filename) {
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

  global_costmap  = NEWMAP16;
  sum_map         = NEWMAP64;
  update_heat_map = NEWMAP16;
  entropy_map     = NEWMAP16;

  copy_int8_vector_to_int16_array(msg.data, global_costmap);

  global_set = true;
}

void costmap_update_handler(const map_msgs::OccupancyGridUpdate& update) {
  if (!global_set) {
    ROS_INFO("Update received without global set");
    return;
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
    uint32_t x_index = i%u_width + u_x;
    uint32_t y_index = i/u_width + u_y;

    // update the update heat map
    update_heat_map[px(x_index,y_index)] += 1;

    // update the sum map
    //sum_map[px(x_index,y_index)] += *it > 50 ? 1 : 0;
    sum_map[px(x_index,y_index)] += *it;

    // TODO: calculate entropy and apply the patch
    // TODO: ensure this actually works and all index calculations are right
    // TODO: find way of displaying matrices as an image
    i++;
  }
}

map16 generate_average_map(const map64 sum, const map16 heat) {
  map16 average = NEWMAP16;

  for (size_t x = 0; x < global_width; x++) {
    for (size_t y = 0; y < global_height; y++) {
      int64_t s = sum[px(x,y)];
      int64_t h = heat[px(x,y)] * 100;
      if (h > 0) {
        float avg = ((float)s) / ((float)h);
        average[px(x,y)] = avg * 100;
      } else {
        average[px(x,y)] = -1;
      }
    }
  }
  return average;
}

map16 deflate(const map16 cmap) {
  map16 deflated = NEWMAP16;

  for (size_t x = 0; x < global_width; x++) {
    for (size_t y = 0; y < global_height; y++) {
      int64 c  = cmap[px(x,y)];
      //int64 cn = cmap[px(x,y+1)];
      //int64 cs = cmap[px(x,y-1)];
      //int64 cw = cmap[px(x-1,y)];
      //int64 ce = cmap[px(x+1,y)];
      
      bool hundred = c == 100;
      //bool edge = (x == 0) || (x == global_width-1) || (y == 0) || (y == global_height-1);
      //bool neighbors = (c == 99) && (cn == 99);// && (cs == 99) && (cw == 99) && (ce == 99);
      //bool check = hundred || (!edge && neighbors);
      deflated[px(x,y)] = hundred ? 255 : 0;
    }
  }

  return deflated;
}

map16 combine_maps(map16 primary, map16 secondary) {
  map16 combined = NEWMAP16;

  for (size_t x = 0; x < global_width; x++) {
    for (size_t y = 0; y < global_height; y++) {
      int64 p = primary[px(x,y)];
      int64 s = secondary[px(x,y)];
      combined[px(x,y)] = p >= 0 ? p : s;
    }
  }
  return combined;
}

map16 invert(map16 cmap) {
  map16 inverted = NEWMAP16;

  for (size_t x = 0; x < global_width; x++) {
    for (size_t y = 0; y < global_height; y++) {
      int64 c = cmap[px(x,y)];
      inverted[px(x,y)] = c == 255 ? 0 : 255;
    }
  }
  return inverted;
}

map16 generate_costmap() {
  map16 res = NEWMAP16;

  map16 average  = generate_average_map(sum_map, update_heat_map);
  map16 combined = combine_maps(average, global_costmap);
  map16 deflated = deflate(combined);
  map16 inverted = invert(deflated);

  map_to_img(combined, "combined.png");
  map_to_img(deflated, "deflated.png");
  map_to_img(deflate(global_costmap), "deflated_global.png");

  delete average;
  delete deflated;
  delete combined;

  return inverted;
}

void generate_results() {
  ROS_INFO("Saving results...");
  time_t theTime = time(NULL);
  struct tm *aTime = localtime(&theTime);

  std::ostringstream oss;
  oss << "/home/users/plankenau/costmap_results/"
      << "[" << (aTime->tm_mday) << "-" << (aTime->tm_mon + 1) << "-" << (aTime->tm_year + 1900)
      << " " << (aTime->tm_hour) << ":" << (aTime->tm_min) << ":" << (aTime->tm_sec) << "] "
      << "_results/";
  std::string path_str = oss.str();

  boost::filesystem::create_directory(path_str);

  map16 result = generate_costmap();

  map_to_img(global_costmap,  path_str + "global_costmap.png");
  map_to_img(update_heat_map, path_str + "update_heat_map.png");
  map_to_img(result, path_str + "result.png");


  ROS_INFO("Results saved!");

  delete result;
}


void ctrlc(int s){
  generate_results();
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "map_generator");
  ros::NodeHandle n;

  signal(SIGINT, ctrlc);

  ros::Subscriber global_sub = n.subscribe("/move_base/global_costmap/costmap", 5, global_costmap_handler);

  ros::Subscriber update_sub = n.subscribe("/move_base/global_costmap/costmap_updates", 1000, costmap_update_handler);

  ROS_INFO("bwi_dyncostmap: Map generator running");
  ros::spin();

  return 0;
}
