#include <pcl_to_grid_map/pcl_to_grid_map.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_to_grid_map_node");
  transform::pcl_to_grid_map::PCLToGridMap pcl_to_grid_map;
  // run
  ros::spin();
  return EXIT_SUCCESS;
}