#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

std::vector<float> CostmapCore::create_costmap(float angle_min, float angle_increment,
      float range_min, float range_max, std::vector<float> ranges) {
    std::vector<float> costmap(rows * cols);

    // stores obstacle points
    std::queue<Point> obstacles;
    int n = ranges.size();
    for (int i = 0; i < n; i++) {
        float angle = angle_min + angle_increment * i;
        float range = ranges[i];

        if (range < range_max && range > range_min) {
            int x = origin_x + std::floor(range * std::cos(angle) / resolution);
            int y = origin_y + std::floor(range * std::sin(angle) / resolution); 

            if (x < cols && y < rows && x > 0 && y > 0) {
                // mark obstacle
                costmap[y * cols + x] = mark_obstacle;  
                obstacles.push({x, y});
            }
        }
    }

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
            double distance = std::hypot(dx, dy) * resolution;

            if (distance < inflation_radius_m) {
              // assign inflated cost to the surrounding cell
              double cost = max_cost * (1 - distance / inflation_radius_m);
              if (costmap[adj_y * cols + adj_x] < cost) {
                costmap[adj_y * cols + adj_x] = cost;
              }
            }
          }
        }
    }

    return costmap;
}

}