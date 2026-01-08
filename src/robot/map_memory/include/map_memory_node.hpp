#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void publish();

  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid last_costmap_;
    nav_msgs::msg::Odometry odom;
    double last_x, last_y;
    const double threshold_m = 1.5;
    bool update_map = false;
    bool costmap_updated = false;

    // global map settings
    const float width_m = 50;
    const float height_m = 50;
    const float resolution = 0.1;
    const float map_origin_x = -width_m / 2.0;
    const float map_origin_y = -height_m / 2.0;
    const int cols = std::ceil(width_m / resolution);
    const int rows = std::ceil(height_m / resolution);

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif 
