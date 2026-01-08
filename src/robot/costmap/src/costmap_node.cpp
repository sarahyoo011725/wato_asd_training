#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

namespace {
  constexpr auto default_test_topic = "/test_topic";
  constexpr auto default_sensor_topic = "/lidar";
  constexpr auto default_costmap_topic = "/costmap";
}

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  string_pub_ = create_publisher<std_msgs::msg::String>(default_test_topic, rclcpp::SystemDefaultsQoS());
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(default_costmap_topic, rclcpp::SystemDefaultsQoS());

  //timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publish_msg, this));

  sensor_msg_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    default_sensor_topic, 
    rclcpp::SystemDefaultsQoS(), 
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      auto m = msg.get();
      auto costmap = costmap_.create_costmap(
        m->angle_min, m->angle_increment, m->range_min, m->range_max, m->ranges);

      nav_msgs::msg::OccupancyGrid nav_msg;
      nav_msg.header.frame_id = "map";
      nav_msg.header.stamp = now();
      nav_msg.info.height = costmap_.height_m;
      nav_msg.info.width = costmap_.width_m;
      nav_msg.info.resolution = costmap_.resolution;
      nav_msg.info.origin.position.x = costmap_.origin_x;
      nav_msg.info.origin.position.y = costmap_.origin_y;
      nav_msg.info.origin.position.z = 0;
      nav_msg.info.origin.orientation.w = 0;
      nav_msg.data.resize(costmap_.rows * costmap_.cols);

      for (int i = 0; i < costmap.size(); i++) {
        if (costmap[i] == 0) {
          nav_msg.data[i] = 0;
        } else {
          nav_msg.data[i] = (int) (std::min(costmap_.mark_obstacle, 
            costmap[i] * costmap_.mark_obstacle / costmap_.max_cost));
        }
      }

      costmap_pub_->publish(nav_msg);
    });
}

void CostmapNode::publish_msg() {
  auto msg = std_msgs::msg::String();
  msg.data = "Test message publishment";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
  string_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}