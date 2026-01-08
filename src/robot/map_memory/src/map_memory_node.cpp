#include "map_memory_node.hpp"

#include <vector>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MapMemoryNode::costmap_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MapMemoryNode::odometry_callback, this, std::placeholders::_1));
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::SystemDefaultsQoS());
  // run a periodic loop at 1s
  timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::publish, this));

  // global map settings
  global_map_.header.frame_id = "map";
  global_map_.info.origin.position.x = -map_origin_x;
  global_map_.info.origin.position.y = -map_origin_y;
  global_map_.info.origin.position.z = 0;
  global_map_.info.width = std::ceil(width_m / resolution); // number of horizontal cells
  global_map_.info.height = std::ceil(height_m / resolution); // number of vertical cells
  global_map_.info.resolution = resolution;
}

void MapMemoryNode::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  last_costmap_ = *msg;
  costmap_updated = true;
}

void MapMemoryNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto m = msg.get();
  odom = *m;
  int x = m->pose.pose.position.x;
  int y = m->pose.pose.position.y;
  int dist = std::hypot(x - last_x, y - last_y);
  if (dist >= threshold_m) {
    last_x = x;
    last_y = y;
    update_map = true;
  }
}

void MapMemoryNode::publish() {
  if (!update_map || !costmap_updated) return;
  const double pose_x = odom.pose.pose.position.x;
  const double pose_y = odom.pose.pose.position.y;
  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;
  double qw = odom.pose.pose.orientation.w;
  // yaw
  double heading = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

  size_t n = last_costmap_.data.size();
  const int costmap_cols = last_costmap_.info.width;
  const int costmap_rows = last_costmap_.info.height; 
  const float costmap_resolution = last_costmap_.info.resolution;

  for (size_t i = 0; i < n; i++) {
    // costmap cell index
    int ix = i % costmap_cols;
    int iy = i / costmap_cols;
    // convert the cell index into pose in robot frame 
    double local_x = last_costmap_.info.origin.position.x + (ix + 0.5) * costmap_resolution;
    double local_y = last_costmap_.info.origin.position.y + (iy + 0.5) * costmap_resolution;
    // convert robot frame cell pose into global frame: rotate by heading and translate by robot pose
    double global_x = pose_x + local_x * std::cos(heading) - local_y * std::sin(heading);
    double global_y = pose_y + local_x * std::sin(heading) + local_y * std::cos(heading);
    // convert the global cell pose into global map index
    int gx = std::lround((global_x - map_origin_x) / resolution);
    int gy = std::lround((global_y - map_origin_y) / resolution);

    if (gx < 0 || gy < 0 || gx >= cols || gy >= rows) continue;

    int8_t cost = last_costmap_.data[i];
    if (cost <= 100 && cost >= 0) {
      // ensure safe overwrite
      global_map_.data[gy * cols + gx] = std::max(global_map_.data[gy * cols + gx], cost);
    }
  }

  global_map_.header.stamp = now();
  map_pub_->publish(global_map_);
  update_map = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
