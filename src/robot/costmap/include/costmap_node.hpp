#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" 
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  struct Point {
    int x, y;
  };

  public:
    CostmapNode();

  private:
    robot::CostmapCore costmap_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sensor_msg_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // costmap settings
    const float width_m = 20;
    const float height_m = 20;
    const float resolution = 0.1;
    const float inflation_radius_m = 1.0;
    const float mark_obstacle = 100;
    const float max_cost = 254;
    const int rows = std::ceil(height_m / resolution);
    const int cols = std::ceil(width_m / resolution);

    std::vector<float> create_costmap(float angle_min, float angle_increment,
      float range_min, float range_max, const std::vector<float> &ranges);
    void sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif 