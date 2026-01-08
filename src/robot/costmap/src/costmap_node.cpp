#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  string_pub_ = create_publisher<std_msgs::msg::String>("/test_topic", rclcpp::SystemDefaultsQoS());
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", rclcpp::SystemDefaultsQoS());
  sensor_msg_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", rclcpp::SystemDefaultsQoS(), std::bind(&CostmapNode::sensor_callback, this, std::placeholders::_1));
}

void CostmapNode::sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  auto m = msg.get();
  auto costmap = create_costmap(m->angle_min, m->angle_increment, m->range_min, m->range_max, m->ranges);

  nav_msgs::msg::OccupancyGrid nav_msg;
  nav_msg.header.frame_id = "map";
  nav_msg.header.stamp = now();
  nav_msg.info.width = cols;
  nav_msg.info.height = rows;
  nav_msg.info.resolution = resolution;
  nav_msg.info.origin.position.x = width_m / 2.0;
  nav_msg.info.origin.position.y = height_m / 2.0;
  nav_msg.info.origin.position.z = 0;
  nav_msg.info.origin.orientation.w = 1.0;
  nav_msg.data.resize(rows * cols);

  for (int i = 0; i < costmap.size(); i++) {
    if (costmap[i] == 0) {
      nav_msg.data[i] = 0;
    } else {
      nav_msg.data[i] = (int) (std::min(100.0f, costmap[i] * 100.0f / max_cost));
    }
  }

  costmap_pub_->publish(nav_msg);
}

std::vector<float> CostmapNode::create_costmap(float angle_min, float angle_increment,
  float range_min, float range_max, const std::vector<float> &ranges) {
  std::vector<float> costmap(rows * cols, 0.0f);
  std::queue<Point> obstacles;
  int n = ranges.size();

  // origin indices in grid
  Point origin;
  origin.x = std::floor(cols / 2);
  origin.y = std::floor(rows / 2);

  for (int i = 0; i < n; i++) {
    double angle = angle_min + angle_increment * i;
    double range = ranges[i];

    if (range <= range_max && range >= range_min) {
      // transform cartesian coordinates into grid indices
      int x = origin.x + std::lround(range * std::cos(angle) / resolution);
      int y = origin.y + std::lround(range * std::sin(angle) / resolution); 

      if (x < cols && y < rows && x >= 0 && y >= 0) {
        // mark obstacle
        costmap[y * cols + x] = max_cost;  
        // stores obstacle points
        obstacles.push({x, y});
      }
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

        if (adj_y >= rows || adj_x >= cols || adj_y < 0 || adj_x < 0) {
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