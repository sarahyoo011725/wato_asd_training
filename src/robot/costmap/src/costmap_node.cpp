#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  string_pub_ = create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  sensor_msg_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::sensor_callback, this, std::placeholders::_1));
}

void CostmapNode::sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  auto m = msg.get();
  auto costmap = create_costmap(m->angle_min, m->angle_increment, m->range_min, m->range_max, m->ranges);

  nav_msgs::msg::OccupancyGrid nav_msg;
  nav_msg.header.frame_id = "sim_world";
  nav_msg.header.stamp = now();
  nav_msg.info.width = cols;
  nav_msg.info.height = rows;
  nav_msg.info.resolution = resolution;
  nav_msg.info.origin.position.x = -width_m / 2.0;
  nav_msg.info.origin.position.y = -height_m / 2.0;
  nav_msg.info.origin.position.z = 0;
  nav_msg.info.origin.orientation.w = 1.0;
  nav_msg.data.resize(rows * cols);

  size_t n = costmap.size();
  for (size_t i = 0; i < n; i++) {
    nav_msg.data[i] = costmap[i]; 
  }

  costmap_pub_->publish(nav_msg);
}

std::vector<float> CostmapNode::create_costmap(float angle_min, float angle_increment,
  float range_min, float range_max, const std::vector<float> &ranges) {
  std::vector<float> costmap(rows * cols, 0.0f);
  std::queue<Point> obstacles;
  size_t n = ranges.size();

  // origin indices in grid
  Point origin;
  origin.x = std::floor(cols / 2);
  origin.y = std::floor(rows / 2);

  for (size_t i = 0; i < n; i++) {
    double angle = angle_min + angle_increment * i;
    double range = ranges[i];

    if (range <= range_max && range >= range_min) {
      // transform cartesian coordinates into grid indices
      int x = origin.x + std::floor(range * std::cos(angle) / resolution);
      int y = origin.y + std::floor(range * std::sin(angle) / resolution); 

      if (x < 0 || x >= cols || y < 0 || y >= rows) {
        //RCLCPP_INFO(get_logger(), "costmap index out of bound: (x=%d, y=%d)", x, y);
        continue;
      }

      // mark obstacle
      costmap[y * cols + x] = max_cost;  
      // stores obstacle points
      obstacles.push({x, y});
    }
  }

  const int step_size = std::ceil(inflation_radius_m / resolution);

  while (!obstacles.empty()) {
    Point obstacle = obstacles.front();
    obstacles.pop();

    // calculate the euclidean distance between an obstacle cell and adjacent cells 
    for (int dy = -step_size; dy <= step_size; dy++) {
      for (int dx = -step_size; dx <= step_size; dx++) {
        int adj_x = obstacle.x + dx;
        int adj_y = obstacle.y + dy;

        if (adj_x < 0 || adj_x >= cols || adj_y < 0 || adj_y >= rows) {
          continue;
        }

        // calculate euclidean distance in meter
        double distance_m = std::hypot(dx, dy) * resolution;

        if (distance_m < inflation_radius_m) {
          // assign inflated cost to the surrounding cell
          float cost = max_cost * (1 - distance_m / inflation_radius_m);
          if (costmap[adj_y * cols + adj_x] < cost) {
            costmap[adj_y * cols + adj_x] = cost;
          }
        }
      }
    }
  }

  return costmap;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}