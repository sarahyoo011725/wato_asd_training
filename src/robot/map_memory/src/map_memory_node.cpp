#include "map_memory_node.hpp"

#include <vector>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      last_costmap_ = *msg;
      costmap_updated = true;
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      auto m = msg.get();
      odom = *m;
      int x = m->pose.pose.position.x;
      int y = m->pose.pose.position.y;
      int cost = std::hypot(x - last_x, y - last_y);
      if (cost >= threshold_meter) {
        last_x = x;
        last_y = y;
        update_map = true;
      }
    });
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::SystemDefaultsQoS());
  timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::publish, this));
}

void MapMemoryNode::publish() {
  if (!update_map || !costmap_updated) return;
  // integrate costmap

  // global width & height
  const float width_m = 100;
  const float height_m = 100;
  const float resolution = 0.1;

  const double pose_x = odom.pose.pose.position.x;
  const double pose_y = odom.pose.pose.position.y;
  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;
  double qw = odom.pose.pose.orientation.w;
  // yaw
  double heading = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

  const double map_origin_x = -width_m / 2.0;
  const double map_origin_y = -height_m / 2.0;
  const int cols = (int)(width_m / resolution);
  const int rows = (int)(height_m / resolution);
  size_t n = last_costmap_.data.size();

  for (int i = 0; i < n; i++) {
    int index_x = i % cols;
    int index_y = i / cols;
    double local_x = (index_x - cols/2) * resolution;
    double local_y = (index_y - rows/2) * resolution;
    // rotate by heading and translate by pose
    double global_x = pose_x + local_x * std::cos(heading) - local_y * std::sin(heading);
    double global_y = pose_y + local_x * std::sin(heading) + local_y * std::cos(heading);

    int gx = std::floor((global_x - map_origin_x) / resolution);
    int gy = std::floor((global_y - map_origin_y) / resolution);

    if (gx < 0 || gy < 0 || gx >= cols || gy >= rows) continue;

    int8_t cost = last_costmap_.data[i];
    if (cost <= 254 && cost >= 0) {
      global_map_.data[gy * cols + gx] = cost;
    }
  }

  global_map_.header.frame_id = "map";
  global_map_.header.stamp = now();
  global_map_.info.origin.position.x = map_origin_x;
  global_map_.info.origin.position.y = map_origin_y;
  global_map_.info.origin.position.z = 0;
  global_map_.info.width = cols;
  global_map_.info.height = rows;
  global_map_.info.resolution = resolution;
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
