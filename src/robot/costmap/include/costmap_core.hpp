#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"

namespace robot
{
struct Point {
  int x, y;
};

class CostmapCore {
  public:
  // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
  explicit CostmapCore(const rclcpp::Logger& logger);
  std::vector<float> create_costmap(float angle_min, float angle_increment,
    float range_min, float range_max, std::vector<float> ranges);

  const float width_m = 100;
  const float height_m = 100;
  const float resolution = 0.1;
  const float inflation_radius_m = 1.0;
  const int rows = (int) (height_m / resolution);
  const int cols = (int) (width_m / resolution);
  const int step_size = (int) (inflation_radius_m / resolution);
  const float mark_obstacle = 100;
  const float max_cost = 254;
  const int origin_x = cols / 2;
  const int origin_y = rows / 2;

  private:

    rclcpp::Logger logger_;
};

}  

#endif  