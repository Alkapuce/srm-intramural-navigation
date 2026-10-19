#include <fstream>
#include <nav2_map_server/map_io.hpp>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  nav_msgs::msg::OccupancyGrid map;
  if (nav2_map_server::loadMapFromYaml(argv[1], map) != nav2_map_server::LOAD_MAP_SUCCESS)
    return 1;
  std::ofstream out(argv[2], std::ios::binary);
  out.write(reinterpret_cast<const char *>(map.data.data()), map.data.size());
  return out ? 0 : 2;
}
